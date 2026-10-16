"""Versioned USB ground station replies. Parameter framing is shared."""
import math

MODES = {"MANUAL": "手动", "STABILIZE": "增稳"}
NUMERIC = ("ms", "roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az",
           "airspeed", "angle", "angle_valid", "locked", "rc1", "rc2", "rc3", "rc4", "rc5", "rc6")


def decode_fields(kind, fields):
    result = {}
    for field in fields:
        key, separator, value = field.partition("=")
        if not separator or not key or key in result:
            raise ValueError("地面站响应字段无效或重复")
        result[key] = value
    if kind == "CAPS":
        required = ("version", "imu", "external", "airspeed", "rotate", "barometer", "radio",
                    "calibration", "sensor_setting", "flight_modes")
        if any(key not in result for key in required) or result["version"] != "1":
            raise ValueError("不支持的地面站协议版本")
    elif kind == "DATA":
        if any(key not in result for key in (*NUMERIC, "mode")) or result["mode"] not in MODES:
            raise ValueError("数据帧不完整或模式无效")
        for key in NUMERIC:
            result[key] = float(result[key])
            if not math.isfinite(result[key]):
                raise ValueError("数据包含非有限数值")
        for key in ("angle_valid", "locked"):
            if result[key] not in (0, 1):
                raise ValueError("状态字段无效")
        if not 0 <= result["ms"] <= 0xffffffff or not result["ms"].is_integer():
            raise ValueError("时间戳无效")
        for key in ("rc7", "rc8", "n1", "n2", "n3", "n4", "n5", "n6", "n7", "n8", "radio_valid"):
            if key in result:
                result[key] = float(result[key])
                if not math.isfinite(result[key]):
                    raise ValueError("遥控数据包含非有限数值")
        if "radio_valid" in result and result["radio_valid"] not in (0, 1):
            raise ValueError("遥控接收状态无效")
    elif kind == "CONFIG":
        required = ("count", "master", "roll_a", "ab", "ac", "bd", "ce", "df", "eg", "left_valid", "right_valid")
        if any(key not in result for key in required):
            raise ValueError("构型数据不完整")
        for key in required:
            result[key] = float(result[key])
            if not math.isfinite(result[key]):
                raise ValueError("构型包含非有限数值")
        if result["count"] not in (1, 3, 4, 5, 7):
            raise ValueError("不支持的机数")
        result["count"] = int(result["count"])
        for key in ("master", "left_valid", "right_valid"):
            if result[key] not in (0, 1):
                raise ValueError("构型状态无效")
        if "aircraft_id" in result or "config_valid" in result:
            if "aircraft_id" not in result or "config_valid" not in result:
                raise ValueError("飞机身份字段不完整")
            identity = float(result["aircraft_id"])
            valid = float(result["config_valid"])
            if not math.isfinite(identity) or not identity.is_integer() or not 0 <= identity <= result["count"] or valid not in (0, 1):
                raise ValueError("飞机身份无效")
            if bool(identity) != bool(valid) or bool(result["master"]) != (identity == 1):
                raise ValueError("飞机身份与构型状态不一致")
            result["aircraft_id"], result["config_valid"] = int(identity), int(valid)
        if any(key.startswith(("r_", "p_", "v_")) for key in result):
            for node in range(result["count"]):
                suffix = chr(97 + node)
                for prefix in ("r", "p", "v"):
                    key = f"{prefix}_{suffix}"
                    if key not in result:
                        raise ValueError("多机姿态字段不完整")
                    result[key] = float(result[key])
                    if not math.isfinite(result[key]):
                        raise ValueError("多机姿态包含非有限数值")
                if result[f"v_{suffix}"] not in (0, 1):
                    raise ValueError("多机姿态有效标志错误")
    elif kind == "RC_SAVE":
        if result != {"status": "SAVED"}:
            raise ValueError("遥控校准保存响应无效")
    elif kind == "REBOOT":
        if result != {"status": "REBOOTING"}:
            raise ValueError("重启响应无效")
    else:
        raise ValueError("未知地面站响应")
    return result
