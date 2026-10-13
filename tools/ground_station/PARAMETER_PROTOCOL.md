# USB 参数协议

参数页位于 `parameter_view.py`，完整地面站入口为 `main.py`。USB Serial 与地面站 `GCS` 指令共用一条连接。完整存储、迁移和开发规则见 [统一参数与持久存储](../../docs/parameter-storage.md)。

当前注册 96 项参数，协议和固件服务最多支持 512 项。表格显示类型、范围和存储位置；双击当前值编辑，Enter 提交、Esc 取消。只读状态和生效方式仍保留在协议元数据中。浮点值采用能恢复 float32 的普通十进制，不使用科学计数法；整数拒绝小数、指数形式及 int32 溢出。列标题支持排序。

参数通过验证后先修改 RAM（重启参数修改的是待启动设置），返回 `OK / PENDING`。每种介质最后一次实际变动后等待 2 秒合并保存；相同值不重置计时，改回原值也作为一次 RAM 变动，仍到期保存，不与已保存值比较。身份、校准和 EEPROM 参数要求锁定；重启参数变动立即提示重启并禁止解锁。保存失败保留 RAM 值，每 2 秒重试，通过异步 `STORAGE` 通知结果。手动修改零偏立即使内存校准无效。USB 断开后继续保存；保存前断电可能丢失修改。地面站重启命令先强制保存，失败则拒绝重启。完整 IMU 校准整组更新 RAM 后共用两秒延迟保存；全部校准字段必须位于同一介质。命令超时表示接收结果未确认，需重新读取，不自动重发 SET。

## 请求与回复

请求为 ASCII 行，末尾 LF；请求 ID 为 uint32。日志中仅 `@COFLY` 前缀行按协议解析。

```text
PARAM READ 1
PARAM SET 2 aircraft_id 2
PARAM SET 3 aircraft_id 1
PARAM HELP
```

回复中 `\t` 代表实际制表符：

```text
@COFLY\t1\tBEGIN\tCOFLY_PARAMS_V8\t74\tSD_READY\tLOADED\t57\t17\t0\t27
@COFLY\t1\tVALUE\tKp_roll_angle\tfloat\t0.25\t0\t10\tAttitude\tRoll angle P\tSD\tIMMEDIATE\tEDITABLE
@COFLY\t1\tVALUE\taircraft_id\tint\t0\t0\t3\tAircraft\tIdentity\tEEPROM\tREBOOT\tEDITABLE
...其余参数...
@COFLY\t1\tEND\t74
@COFLY\t2\tOK\taircraft_id\tint\t2\tPENDING
@COFLY\t4\tOK\tKp_roll_angle\tfloat\t0.3\tPENDING
@COFLY\t0\tSTORAGE\tEEPROM\tSAVED
@COFLY\t0\tSTORAGE\tSD\tSAVED
@COFLY\t3\tERROR\tLOCK_REQUIRED\tLock aircraft before changing identity or calibration
```

`BEGIN` 的首字段为固定文件版本 `COFLY_PARAMS_V8`，不再包含板卡或宏配置名称；三个统计字段依次为所选 SD 文件的匹配项、未匹配项和跳过旧项。未匹配项可能来自 EEPROM 或默认值。最后一字段为固件参数名称上限（默认 27），电脑按此上限校验导出名称，不填充字段。SD 和 USB 回复不再携带代次。

`VALUE` 为名称、类型、当前/待启动值、最小值、最大值、分组、说明、`SD|EEPROM`、`IMMEDIATE|REBOOT`、`EDITABLE|READ_ONLY`。电脑端仍支持以前未携带后三项元数据的格式。

`OK` 依次携带名称、类型、当前值、`SAVED|PENDING`。`PENDING` 表示已接受，相关介质等待合并保存；`SAVED` 表示相关介质没有待保存变动。地面站也兼容旧固件的 `RAM_ONLY`。范围、只读和锁定验证失败不修改内存。错误包含 `UNKNOWN_PARAMETER`、`RANGE`、`READ_ONLY`、`LOCK_REQUIRED`、`NO_SD`、`SAVE_FAILED` 等。异步通知使用请求 ID 0：`@COFLY\t0\tSTORAGE\tSD|EEPROM\tSAVED|FAILED`；其中 `SAVED` 才表示本次合并写入及校验成功，`FAILED` 表示保留待保存状态并在 2 秒后重试。`BEGIN` 的来源状态 `PENDING` 表示仍有待写入参数。

`GCS CONFIG` 返回当前生效的 `aircraft_id` 和 `config_valid`，以及构型数量/姿态。参数页的编号/数量是待启动值，飞行页显示当前生效身份。

## 文件与运行成本

SD 主文件 `params.cfg`、回退文件 `params_backup.cfg`、EEPROM 槽与电脑文件均使用同一 V8 ASCII/LF 文本容器：

```text
COFLY_PARAMS_V8
COUNT=2
gain:float=0.25
pwm_rev:int=-1
CRC32=xxxxxxxx
```

记录为 `name:float=value` 或 `name:int=value`，末尾 LF，没有名称补零或持久化 NUL。float32 使用可原样恢复数值的最短普通十进制；头部仅有固定版本和数量，不带 ID、代次、板卡名称及额外完成标记。CRC 尾部 15 字节，覆盖之前全部文本。

名称上限默认 27，每条占用随名称和数值变化，不再有固定偏移。EEPROM 主备槽各 2048 字节，有效文本不超过 2047 字节；SD 文件缓冲 103168 字节，文件须小于该容量，全表最多 512 项。均主数据优先、失败回退备份；写入前备份旧主数据，从备份恢复则直接修复主数据。

电脑导入允许在其他检查通过后确认 CRC 不匹配；固件读取介质仍严格校验 CRC。V7 二进制和其他版本的旧文本不读取，升级后需重新生成文件并设置、校准。详见存储说明。

参数服务静态预留 512 项缓存，SD 文件缓冲由记录兼容长度自动推导；参数名默认上限 27，可在 `src/parameters/parameter_storage_config.h` 配置为 1～128，地面站兼容到 128 字符。控制运算直接读取 RAM 变量；USB 每轮最多接收 64 字节或发送一行。2 秒等待不阻塞主循环；到期保存、CRC、读回校验仍同步执行，会延长该控制周期。本版尚未真机验证最坏周期。

## 构建与验证

在本目录执行 `./build.ps1`，生成 `dist/CoFlyGroundStation.exe`。已通过的自动测试使用本机 socket 模拟飞控，覆盖 V8 文本文件与 CRC、USB 参数导入和失败处理。按项目要求，测试脚本在验证后删除。[旧版板上记录](BOARD_TEST_REPORT.md) 不代表本版已完成真机验证。

## 遥控器整组校准保存

`CAPS` 增加 `radio_calibration=SUPPORTED`，`DATA` 增加 `radio_valid=0/1`（主通道近期有效接收，包含 SBUS 失控/丢帧检查）。九步采样在地面站完成，确认前不改动飞控参数。

命令：`GCS RC_SAVE id min1 max1 trim1 rev1 min2 max2 trim2 rev2 min3 max3 trim3 rev3 min4 max4 trim4 rev4`。严格要求 16 个数值，分别绑定 CH1–4 的 MIN/MAX/TRIM/REV，范围以注册表为准；每轴总行程至少 400 μs，中位与端点距离必须大于现有死区。保留 DZ 和辅助通道。必须锁定且接收机有效，整组验证后同步写入一次 SD 快照，读回校验成功才更新 RAM。

成功：`@COFLY\tid\tRC_SAVE\tstatus=SAVED`，不使用普通逐项 SET 的 PENDING 状态。失败：`ERROR / LOCK_REQUIRED`、`RADIO_INVALID` 或 `SAVE_FAILED`（参数无效或 SD 保存失败）。失败不应用新的遥控 RAM 参数。保存请求超时为 10 秒；超时不能推断板上保存失败，应重新读取核对。该操作同时持久化快照中已有的其他待保存 SD 参数。
