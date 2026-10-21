# 参数模块

固件参数实现集中在本目录。`platformio.ini` 为普通和调试环境统一配置 `src` 为头文件搜索根目录，引用使用 `parameters/parameter_service.h` 等完整模块路径。

| 文件 | 职责 |
| --- | --- |
| `control_parameters.h/.cpp` | 控制参数的 RAM 变量与默认值，控制算法直接读取 |
| `parameter_registry.h/.cpp` | 参数类型、范围、名称、变量绑定、存储介质、生效方式及访问权限 |
| `parameter_keys.h` | 固件内部查询编号；编号不进入持久化文件 |
| `parameter_storage_config.h` | 名称、记录缓冲、参数数量上限及 EEPROM 槽地址/大小 |
| `parameter_record.h/.cpp` | 单条 `name:type=value` 编解码和精确 float32 文本转换 |
| `parameter_text.h/.cpp` | SD/EEPROM 共用的 `COFLY_PARAMS` 文本容器（含飞机总数、编号）、记录计数和 CRC32 校验 |
| `parameter_sd.h/.cpp` | SD 主文件与备份的读取、保存、读回校验及恢复 |
| `parameter_eeprom.h/.cpp` | EEPROM 主槽与备份槽的读取、保存、校验及恢复 |
| `parameter_save_debounce.h` | 每种介质独立的 2 秒延迟保存与失败重试计时 |
| `imu_calibration_integrity.h` | 完整 IMU 校准组的指纹校验 |
| `actuator_parameter_validation.h` | 五路 PWM 上下限的整份候选快照校验 |
| `parameter_service.h/.cpp` | 初始化、参数修改校验、RAM 更新、保存调度、校准组提交和 USB 协议 |

添加或调整参数时，优先修改 `parameter_registry.cpp`。修改每行的 `storage` 可选择 SD 或 EEPROM；IMU 校准组必须整体使用同一介质。控制参数默认值在 `control_parameters.cpp`，飞机身份和传感器变量仍由对应硬件模块持有。

EEPROM 参数仅在锁定时可修改；普通 SD 参数在锁定或解锁时均可修改。参数修改先更新 RAM；实际发生变化时，将对应介质标为 dirty，并重置该介质的 2 秒计时。同值写入不重置计时。EEPROM 与 SD 均在到期后保存当前完整快照；SD 在解锁时也会保存，强制刷新同样可写入。失败保留 dirty 并在 2 秒后重试，SD 重试不受锁定状态限制。重启命令要求锁定并强制保存待写数据。重启生效参数一旦变化，重启提示与禁止解锁状态保持至设备重启。专用 IMU/遥控校准流程仍要求锁定。

新文件第一行以空格分隔 `COFLY_PARAMS`、`AIRCRAFT_COUNT` 与 `AIRCRAFT_ID`，第二行记录参数条数，CRC 覆盖整个头部；兼容加载原 V8 及飞机信息分三行的文本并迁移保存。USB `BEGIN` 的文件标识同步更新，其他参数记录和读写命令不变。PC 地面站的匹配实现位于 `tools/ground_station/parameter_file.py`。存储格式、容量和迁移行为详见 [参数存储说明](../../docs/parameter-storage.md)。

固件使用 `pio run -e teensy41 -e teensy41_debug` 构建。参数服务与 EEPROM 主备故障测试已完成验证；按项目要求，测试脚本在验证后删除。

八路遥控归一化参数为 `RC1_MIN/MAX/TRIM/REV/DZ` 至 `RC8_MIN/MAX/TRIM/REV/DZ`，直接绑定 `radio_comm` 的参数，全部位于 SD，可通过普通参数读写接口修改。

五路执行器软件限幅为整数 SD 参数 `pwm_servo1_min_us/max_us` 至 `pwm_servo5_min_us/max_us`，默认每路 1000～2000 us。`pwm_channel1_mode` 至 `pwm_channel5_mode` 默认均为 0：模式 0 接收 -1～1，按 min/绝对 trim/max 三点插值；模式 1 接收 0～1，按 min/max 两点插值。五路统一调用 `actuatorCommandToPwm`，调用方负责匹配范围；当前舵面应使用 mode=0，油门必须设置并保存 mode=1，不再有油门专用转换函数。模式 0 要求 min<=trim<=max，模式 1 的 trim 仅为安全位置、可在解锁范围外（仍需在驱动范围内）。锁定或解锁时均可修改并立即生效，最后变动满 2 秒后保存；行程每周期使用当前参数，不缓存。各路锁定输出均直接使用绝对 trim（默认 1500）；普通单向电调可设置油门 mode=1、trim=1000。旧 SD 文件缺少模式时补默认 0，因此油门模式须核对；已存上下限倒置或模式 0 中位越界时整组限值恢复默认并迁移保存。旧相对偏置 trim 须按 1500+旧值手动转换后保存，不自动迁移。

本机俯仰算法由整数 SD 参数 `pitch_controller` 选择：0 PID、1 INDI，默认 1；增稳模式使用此选择，修改下一周期生效。各机独立设置，原 `TESTINDI` 宏已移除。控制历史状态由 `control/` 下各控制器对象持有，模式切换由 `control/control_modes.cpp` 管理。

`V_cruise` 为 float32 SD 参数，默认 13 m/s，允许范围 0.1～100 m/s。偏航协调转弯计算直接读取该参数，修改后下一控制周期生效；通过现有参数保存/加载流程持久化，旧文件缺少该项时保留默认值。

增稳空速 PI 使用独立的 `airspeed_control_enabled`、`airspeed_min/max`、`throttle_cruise`、`Kp_airspeed`、`Ki_airspeed` 和 `airspeed_lpf_hz` SD 参数；源码默认关闭（0），设为 1 后启用。范围、回退和调参说明见 [简单空速控制](../../docs/airspeed-control.md)。

执行器调试：整数 SD 参数 `usb_actuator_output_debug` 默认 0，设为 1 后立即启用 `printActuatorOutput(int enabled)`，以独立的 10 Hz 节流打印本机 `ActuatorOutput[5]`。字段顺序为 ail1、ail2、ele、thro、rudd，单位 us；打印点位于最终限幅、锁定覆盖和舵机写出之后，配置无效时打印本机安全位置。USB 与 X6 数传独立发送，忙时跳过该链路，不等待发送空间。参数通过既有延迟保存机制持久化。

空速调试：整数 SD 参数 `usb_airspeed_debug` 默认 0，仅接受 0/1。设为 1 后立即启用 `printAirspeed(int enabled)`，在空速采样之后以独立的 10 Hz 节流打印 `[AIRSPEED] airspeed=... (m/s)`；锁定或飞机身份未配置时也可查看。USB 与 X6 数传独立发送，忙时跳过该链路，不等待发送空间，不额外读取传感器。参数通过现有延迟保存机制持久化，旧参数文件缺少该项时默认关闭。

空速输出同时包含 `rx`（最近一次接收字节数）、`read_ok`（是否收到完整4字节）、`status`（首字节高2位，未读到完整帧时为255）、`raw`（14位压力计数）、`pressure/zero/dp`（压力、校零值、未截断差压，Pa）、`temp`（℃）、`reads/failed`（完整帧/不完整帧累计次数，包含上电校零）。这些字段为已有采样结果的快照，不增加 I²C 请求；`read_ok=1` 只说明帧完整，不代表传感器状态正常或数据新鲜。读取失败时压力和温度仍可能是旧值，需结合 `read_ok` 判断。空速为0不能单独证明传感器通信正常；无风、负差压截零和读失败都可能表现为0。

控制模式调试：整数 SD 参数 `usb_control_mode_debug` 默认 0，设为 1 后立即启用 `printControlMode(int enabled)`，以独立的 10 Hz 节流打印 `initializeInitialControlMode()` 涉及变量的当前值：currentFlightMode、previousFlightMode、activePitchController、previousPitchController、mixerStrategyCommand、configurationControlScale、configurationAngle（rad）、brakeEnabled、controlModeChanged，并附加积分参数 integralControlEnabled。FlightMode 数值 0/1 分别表示手动/增稳；PitchController 数值 0/1 分别表示 PID/INDI。USB 与 X6 数传独立发送，忙时跳过该链路，参数沿用 SD 延迟保存。

积分开关：`integralControlEnabled` 为本机 SD 整数参数，默认 1，仅接受 0/1。0 关闭滚转、俯仰、构型和空速积分并清除历史，1 开启积分；偏航积分独立，不受此开关控制，设 `Ki_yaw_rate=0` 时清除偏航 I 项。修改后下一控制周期生效，使用现有延迟保存机制。该参数不由模式初始化或模式切换覆盖，各机独立设置。

积分限幅：`i_limit_roll_angle`、`i_limit_roll_rate`、`i_limit_pitch_angle`、`i_limit_pitch_rate`、`i_limit_FLAP_RATE` 限制对应控制环乘 Ki 之前的误差积分，默认 45；`i_limit_yaw_rate` 限制乘 Ki 后的 I 项输出，默认 0.1。六项均为 float32 SD 参数，允许范围 0～1000，限制范围为各自参数的正负值，下一控制周期生效，并使用现有延迟保存机制。六路构型角速度环共用 `i_limit_FLAP_RATE`，积分历史各自独立。旧公共 `i_limit` 已移除，加载时跳过，新项缺失时使用各自默认值，迁移保存时写入新项并移除旧项。偏航的新控制律直接相加 FF、P、I、D，不再整体乘 0.573；原有同名增益和 `i_limit_yaw_rate` 已保存值仍会恢复，不自动换算，需要按新尺度检查实际参数。空速 PI 仍独立限制最终 I 项为 ±0.3。

Debug 组保留 `usb_*_debug` 参数名以兼容已有 SD 文件，但所有诊断（含命令链、传感器链及 `debug_print` 函数）均同时提供给 USB 与 Serial8/X6，不依赖 USB 在线。诊断先组装完整帧，再交给对应会话的非阻塞发送游标，每周期最多发送 64 字节；等待中的命令、参数列表和存储通知优先，诊断不会插入协议回复中间。链路忙时独立丢弃新诊断帧，数传带宽不足时不保证完整采样率。
