"""PC snapshots use the same ASCII/LF V8 SD format and float32 codec."""
from dataclasses import replace
import re
import zlib

from protocol import format_value, validate_edit

FILE_LIMIT = 103168
FILE_VERSION = "COFLY_PARAMS_V8"


class CrcMismatchError(ValueError):
    """A well-formed CRC trailer does not match the file body."""


def encode_sd_file(parameters, name_limit=27):
    if not 1 <= name_limit <= 128:
        raise ValueError("参数名称上限无效")
    rows = [p for p in parameters.values() if p.storage == "SD"]
    if not rows or len(rows) > 512:
        raise ValueError("没有可保存的 SD 参数")
    text = f"{FILE_VERSION}\nCOUNT={len(rows)}\n"
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
        raise ValueError("参数文件必须为 ASCII V8 格式") from exc
    if len(lines) < 4 or any(not line.endswith("\n") or "\r" in line or "\0" in line for line in lines):
        raise ValueError("参数文件必须使用 LF 换行，不含 BOM")
    if not re.fullmatch(r"CRC32=[0-9a-fA-F]{8}\n", lines[-1]):
        raise ValueError("CRC32 文件尾无效")
    if not allow_crc_mismatch and zlib.crc32("".join(lines[:-1]).encode("ascii")) != int(lines[-1][6:14], 16):
        raise CrcMismatchError("参数文件 CRC32 校验失败")
    if lines[0] != FILE_VERSION + "\n":
        raise ValueError("参数文件版本不支持；请重新导出 V8 文本文件")
    if not re.fullmatch(r"COUNT=[0-9]+\n", lines[1]):
        raise ValueError("参数数量无效")
    declared = int(lines[1][6:])
    sd_names = {name for name, p in parameters.items() if p.storage == "SD"}
    if not 0 < declared <= 512 or declared != len(lines) - 3:
        raise ValueError("参数文件缺行或数量不匹配")
    values = {}
    for line in lines[2:-1]:
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
