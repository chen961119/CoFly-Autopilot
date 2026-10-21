"""PC snapshots share the firmware's ASCII/LF container and float32 codec."""
from dataclasses import replace
import re
import zlib

from protocol import format_value, validate_edit

FILE_LIMIT = 103168
FILE_VERSION = "COFLY_PARAMS"
LEGACY_FILE_VERSION = "COFLY_PARAMS_V8"
SUPPORTED_FILE_VERSIONS = (FILE_VERSION, LEGACY_FILE_VERSION)


class CrcMismatchError(ValueError):
    """A well-formed CRC trailer does not match the file body."""


def _validate_aircraft_metadata(total, identity):
    if total not in (1, 3, 4, 5, 7) or not 0 <= identity <= total or identity != int(identity):
        raise ValueError("飞机总数或编号无效（编号 0 表示未配置）")


def encode_sd_file(parameters, name_limit=27):
    if not 1 <= name_limit <= 128:
        raise ValueError("参数名称上限无效")
    rows = [p for p in parameters.values() if p.storage == "SD"]
    if not rows or len(rows) > 512:
        raise ValueError("没有可保存的 SD 参数")
    identity = parameters.get("aircraft_id")
    if identity is None or identity.dtype != "int":
        raise ValueError("缺少飞机编号信息，请重新读取完整参数")
    _validate_aircraft_metadata(identity.maximum, identity.value)
    text = (f"{FILE_VERSION} AIRCRAFT_COUNT={int(identity.maximum)} "
            f"AIRCRAFT_ID={int(identity.value)}\nCOUNT={len(rows)}\n")
    names = set()
    for p in rows:
        if p.dtype not in ("float", "int") or p.name in names:
            raise ValueError("参数类型无效或名称重复")
        names.add(p.name)
        if not re.fullmatch(r"[A-Za-z0-9_]{1,128}", p.name) or len(p.name) > name_limit:
            raise ValueError("参数名称无效")
        value = validate_edit(replace(p, read_only=False), format_value(p.value, p.dtype))
        text += f"{p.name}:{p.dtype}={format_value(value, p.dtype)}\n"
    body = text.encode("ascii")
    data = body + f"CRC32={zlib.crc32(body):08x}\n".encode("ascii")
    if len(data) >= FILE_LIMIT:
        raise ValueError("参数文件超过固件容量限制")
    return data


def decode_sd_file(data, parameters, *, allow_crc_mismatch=False):
    if not data or len(data) >= FILE_LIMIT:
        raise ValueError("文件为空或超过固件容量限制")
    try:
        lines = data.decode("ascii").splitlines(keepends=True)
    except UnicodeDecodeError as exc:
        raise ValueError("参数文件必须为 ASCII COFLY_PARAMS 格式") from exc
    if len(lines) < 4 or any(not line.endswith("\n") or "\r" in line or "\0" in line for line in lines):
        raise ValueError("参数文件必须使用 LF 换行，不含 BOM")
    if not re.fullmatch(r"CRC32=[0-9a-fA-F]{8}\n", lines[-1]):
        raise ValueError("CRC32 文件尾无效")
    if not allow_crc_mismatch and zlib.crc32("".join(lines[:-1]).encode("ascii")) != int(lines[-1][6:14], 16):
        raise CrcMismatchError("参数文件 CRC32 校验失败")
    metadata = re.fullmatch(rf"{FILE_VERSION} AIRCRAFT_COUNT=([0-9]+) AIRCRAFT_ID=([0-9]+)\n", lines[0])
    if metadata:
        _validate_aircraft_metadata(int(metadata[1]), int(metadata[2]))
        count_line = 1
    elif lines[0] == FILE_VERSION + "\n":
        if (len(lines) < 6 or not re.fullmatch(r"AIRCRAFT_COUNT=[0-9]+\n", lines[1]) or
                not re.fullmatch(r"AIRCRAFT_ID=[0-9]+\n", lines[2])):
            raise ValueError("飞机总数或编号文件头无效")
        _validate_aircraft_metadata(int(lines[1][15:]), int(lines[2][12:]))
        count_line = 3
    elif lines[0] == LEGACY_FILE_VERSION + "\n":
        count_line = 1
    else:
        raise ValueError("参数文件格式不支持；请重新导出 COFLY_PARAMS 文本文件")
    if not re.fullmatch(r"COUNT=[0-9]+\n", lines[count_line]):
        raise ValueError("参数数量无效")
    declared = int(lines[count_line][6:])
    sd_names = {name for name, p in parameters.items() if p.storage == "SD"}
    if not 0 < declared <= 512 or declared != len(lines) - count_line - 2:
        raise ValueError("参数文件缺行或数量不匹配")
    values = {}
    # Header identity describes the source snapshot; it never changes the target
    # aircraft's identity or compiled aircraft count during an SD import.
    for line in lines[count_line + 1:-1]:
        match = re.fullmatch(r"([A-Za-z0-9_]{1,128}):(float|int)=([^\n]+)\n", line)
        if not match:
            raise ValueError("参数记录格式无效")
        name, dtype, text = match.groups()
        if len(text) >= 64 or (dtype == "float" and not re.fullmatch(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?", text)):
            raise ValueError("参数数值格式或长度无效")
        if name in values or name not in sd_names or parameters[name].dtype != dtype:
            raise ValueError(f"重复、未知、EEPROM 或类型不匹配的参数：{name}")
        p = parameters[name]
        value = validate_edit(replace(p, read_only=False), text)
        if dtype == "float" and value == 0 and any(c in "123456789" for c in text.lower().split("e", 1)[0]):
            raise ValueError("参数数值小于 float32 可表示范围")
        if p.read_only and format_value(value, dtype) != format_value(p.value, dtype):
            raise ValueError(f"不能导入不同的只读参数：{name}")
        values[name] = value
    if set(values) != sd_names:
        raise ValueError("文件与当前 SD 参数表不完整匹配；请读取当前飞控参数后重试")
    return values
