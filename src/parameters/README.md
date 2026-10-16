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

本机俯仰算法由整数 SD 参数 `pitch_controller` 选择：0 PID、1 INDI，默认 1；两种增稳模式均使用此选择，修改下一周期生效。各机独立设置，原 `TESTINDI` 宏已移除。控制历史状态由 `control/` 下各控制器对象持有，模式切换由 `control/control_modes.cpp` 管理。

`V_cruise` 为 float32 SD 参数，默认 13 m/s，允许范围 0.1～100 m/s。偏航协调转弯计算直接读取该参数，修改后下一控制周期生效；通过现有参数保存/加载流程持久化，旧文件缺少该项时保留默认值。

执行器调试：整数 SD 参数 `usb_actuator_output_debug` 默认 0，设为 1 后立即启用 `printActuatorOutput(int enabled)`，以独立的 10 Hz 节流打印本机 `ActuatorOutput[5]`。字段顺序为 ail1、ail2、ele、thro、rudd，单位 us；打印点位于最终限幅、锁定覆盖和舵机写出之后，配置无效时打印本机安全位置。USB 未连接或发送缓冲不足时跳过，不等待发送空间。参数通过既有延迟保存机制持久化。

控制模式调试：整数 SD 参数 `usb_control_mode_debug` 默认 0，设为 1 后立即启用 `printControlMode(int enabled)`，以独立的 10 Hz 节流打印 `initializeInitialControlMode()` 涉及变量的当前值：currentMode、lastMode、pitchController、lastPitchController、mixerStrategy、configurationCoeff、configurationAngle、brake、changed、integralEnabled。FlightMode 数值 0/1/2 分别表示手动/无积分增稳/增稳；PitchController 数值 0/1 分别表示 PID/INDI。USB 未连接或发送缓冲不足时跳过，参数沿用 SD 延迟保存。
