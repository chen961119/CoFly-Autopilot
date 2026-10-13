# 参数模块

固件参数实现集中在本目录。`platformio.ini` 为普通和调试环境统一配置 `src` 为头文件搜索根目录，引用使用 `parameters/parameter_service.h` 等完整模块路径。

| 文件 | 职责 |
| --- | --- |
| `control_parameters.h/.cpp` | 控制参数的 RAM 变量与默认值，控制算法直接读取 |
| `parameter_registry.h/.cpp` | 参数类型、范围、名称、变量绑定、存储介质、生效方式及访问权限 |
| `parameter_keys.h` | 固件内部查询编号；编号不进入持久化文件 |
| `parameter_storage_config.h` | 名称、记录缓冲、参数数量上限及 EEPROM 槽地址/大小 |
| `parameter_record.h/.cpp` | 单条 `name:type=value` 编解码和精确 float32 文本转换 |
| `parameter_text.h/.cpp` | SD/EEPROM 共用的 V8 文本容器、记录计数和 CRC32 校验 |
| `parameter_sd.h/.cpp` | SD 主文件与备份的读取、保存、读回校验及恢复 |
| `parameter_eeprom.h/.cpp` | EEPROM 主槽与备份槽的读取、保存、校验及恢复 |
| `parameter_save_debounce.h` | 每种介质独立的 2 秒延迟保存与失败重试计时 |
| `imu_calibration_integrity.h` | 完整 IMU 校准组的指纹校验 |
| `parameter_service.h/.cpp` | 初始化、参数修改校验、RAM 更新、保存调度、校准组提交和 USB 协议 |

添加或调整参数时，优先修改 `parameter_registry.cpp`。修改每行的 `storage` 可选择 SD 或 EEPROM；IMU 校准组必须整体使用同一介质。控制参数默认值在 `control_parameters.cpp`，飞机身份和传感器变量仍由对应硬件模块持有。

参数修改先更新 RAM；实际发生变化时，将对应介质标为 dirty，并重置该介质的 2 秒计时。同值写入不重置计时。到期保存当前完整快照，失败保留 dirty 并在 2 秒后重试。重启命令会强制保存待写数据。重启生效参数一旦变化，重启提示与禁止解锁状态保持至设备重启。

本次整理不改变 V8 文本格式或 USB 协议。PC 地面站的匹配实现位于 `tools/ground_station/parameter_file.py`。存储格式、容量和迁移行为详见 [参数存储说明](../../docs/parameter-storage.md)。

固件使用 `pio run -e teensy41 -e teensy41_debug` 构建。参数服务与 EEPROM 主备故障测试已完成验证；按项目要求，测试脚本在验证后删除。

八路遥控归一化参数为 `RC1_MIN/MAX/TRIM/REV/DZ` 至 `RC8_MIN/MAX/TRIM/REV/DZ`，直接绑定 `radio_comm` 的参数，全部位于 SD，可通过普通参数读写接口修改。
