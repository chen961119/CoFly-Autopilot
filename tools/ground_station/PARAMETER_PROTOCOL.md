# USB 参数协议

参数页位于 `parameter_view.py`，完整地面站入口为 `main.py`。USB Serial 与地面站 `GCS` 指令共用一条连接。完整存储、迁移和开发规则见 [统一参数与持久存储](../../docs/parameter-storage.md)。

当前注册 111 项参数，协议和固件服务最多支持 512 项。表格显示类型、范围和存储位置；双击当前值编辑，Enter 提交、Esc 取消。只读状态和生效方式仍保留在协议元数据中。浮点值采用能恢复 float32 的普通十进制，不使用科学计数法；整数拒绝小数、指数形式及 int32 溢出。列标题支持排序。

参数通过验证后先修改 RAM（重启参数修改的是待启动设置），返回 `OK / PENDING`。每种介质最后一次实际变动后等待 2 秒合并保存；相同值不重置计时，改回原值也作为一次 RAM 变动，仍到期保存，不与已保存值比较。普通 SD 参数在锁定或解锁时均可修改并到期保存；EEPROM 参数修改和专用校准流程要求锁定；重启参数变动立即提示重启并禁止解锁。保存失败保留 RAM 值，每 2 秒重试，通过异步 `STORAGE` 通知结果。手动修改零偏立即使内存校准无效。USB 断开后继续保存；保存前断电可能丢失修改。地面站重启命令先强制保存，失败则拒绝重启。完整 IMU 校准整组更新 RAM 后共用两秒延迟保存；全部校准字段必须位于同一介质。命令超时表示接收结果未确认，需重新读取，不自动重发 SET。

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
@COFLY\t1\tBEGIN\tCOFLY_PARAMS\t74\tSD_READY\tLOADED\t57\t17\t0\t27
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

`BEGIN` 的首字段为固定文件标识 `COFLY_PARAMS`，不包含板卡或宏配置名称；三个统计字段依次为所选 SD 文件的匹配项、未匹配项和跳过旧项。未匹配项可能来自 EEPROM 或默认值。最后一字段为固件参数名称上限（默认 27），电脑按此上限校验导出名称，不填充字段。SD 和 USB 回复不再携带代次。

`VALUE` 为名称、类型、当前/待启动值、最小值、最大值、分组、说明、`SD|EEPROM`、`IMMEDIATE|REBOOT`、`EDITABLE|READ_ONLY`。电脑端仍支持以前未携带后三项元数据的格式。

`OK` 依次携带名称、类型、当前值、`SAVED|PENDING`。`PENDING` 表示已接受，相关介质等待合并保存；`SAVED` 表示相关介质没有待保存变动。地面站也兼容旧固件的 `RAM_ONLY`。范围、只读和锁定验证失败不修改内存。错误包含 `UNKNOWN_PARAMETER`、`RANGE`、`READ_ONLY`、`LOCK_REQUIRED`、`NO_SD`、`SAVE_FAILED` 等。异步通知使用请求 ID 0：`@COFLY\t0\tSTORAGE\tSD|EEPROM\tSAVED|FAILED`；其中 `SAVED` 才表示本次合并写入及校验成功，`FAILED` 表示保留待保存状态并在 2 秒后重试。`BEGIN` 的来源状态 `PENDING` 表示仍有待写入参数。

`GCS CONFIG` 返回当前生效的 `aircraft_id` 和 `config_valid`，以及构型数量/姿态。参数页的编号/数量是待启动值，飞行页显示当前生效身份。

## 文件与运行成本

SD 主文件 `params.cfg`、回退文件 `params_backup.cfg`、EEPROM 槽与电脑文件均使用同一 ASCII/LF 文本容器：

```text
COFLY_PARAMS AIRCRAFT_COUNT=3 AIRCRAFT_ID=2
COUNT=2
gain:float=0.25
pwm_rev:int=-1
CRC32=xxxxxxxx
```

记录为 `name:float=value` 或 `name:int=value`，末尾 LF，没有名称补零或持久化 NUL。float32 使用可原样恢复数值的最短普通十进制；第一行以空格分隔 `COFLY_PARAMS`、`AIRCRAFT_COUNT`、`AIRCRAFT_ID`，第二行为 `COUNT`；飞机总数来自编译配置，编号来自保存快照中的 `aircraft_id`（0 表示未配置）。飞机元数据只记录来源，导入时不会据此改变目标身份或编译数量；不带内部参数 ID、代次、板卡名称及额外完成标记。CRC 尾部 15 字节，覆盖之前全部文本。

名称上限默认 27，每条占用随名称和数值变化，不再有固定偏移。EEPROM 主备槽各 2048 字节，有效文本不超过 2047 字节；SD 文件缓冲 103168 字节，文件须小于该容量，全表最多 512 项。均主数据优先、失败回退备份；写入前备份旧主数据，从备份恢复则直接修复主数据。

电脑导入允许在其他检查通过后确认 CRC 不匹配；固件读取介质仍严格校验 CRC。旧 `COFLY_PARAMS_V8` 文本及此前飞机信息分三行的文本仍可读取，飞控自动迁移、电脑重新导出时生成新格式；V7 二进制和其他旧格式不读取。详见存储说明。

参数服务静态预留 512 项缓存，SD 文件缓冲由记录兼容长度自动推导；参数名默认上限 27，可在 `src/parameters/parameter_storage_config.h` 配置为 1～128，地面站兼容到 128 字符。控制运算直接读取 RAM 变量；USB 每轮最多接收 64 字节或发送一行。2 秒等待不阻塞主循环；到期保存、CRC、读回校验仍同步执行，会延长该控制周期。本版尚未真机验证最坏周期。

## USB SD 文件协议 V1

`GCS CAPS` 增加 `sd_files=USB_V1` 和 `sd_block=4096`（支持的最大分块字节数）。复用 `@COFLY` 行帧与请求 ID，SD 命令只接受第一 USB CDC，普通/双串口调试固件一致，X6 不支持。除 `CANCEL` 外，每条命令都在飞控检查 `isFlightLocked()` 和 SD 就绪状态。所有数据和 UTF-8 路径/名称采用十六进制编码，避免换行、空格和普通诊断输出破坏行帧。绝对路径最多 255 字节，拒绝 `.`、`..`、空路径分量和反斜杠。

| 命令 | 回复类型和字段（以实际制表符分隔） |
| --- | --- |
| `SD LIST id path_hex` | `SD_BEGIN`，打开目录并替换之前的文件/目录 |
| `SD NEXT id` | `SD_ENTRY name_hex size modified DIR\|FILE`；到末尾为 `SD_END` |
| `SD OPEN id path_hex` | `SD_OPEN size`，只读打开文件，固定大小快照，偏移与 CRC 归零 |
| `SD READ id offset [block_size]` | `SD_DATA offset data_hex crc32`；到固定大小末尾为 `SD_EOF size crc32`；分块省略时为 256，指定时允许 1–4096 |
| `SD KEEP id` | `SD_KEEP`，维持当前会话 |
| `SD CANCEL id` | `SD_CANCEL`，关闭句柄并恢复日志/延迟 SD 保存，重复取消安全 |

`modified` 为 SD 原有本地时间 `YYYY-MM-DDTHH:MM:SS`，无有效时间为 `-`；大小和偏移为 uint64 十进制。新地面站按能力声明请求最多 4096 字节；旧固件没有 `sd_block` 时仍发送原四字段 READ，使用 256 字节；旧地面站连接新固件也保持 256 字节。行帧及隐藏后台流量的缓冲上限同步增至 8320 字节，USB 文件会话期间每控制周期最多发送 2048 字节且不超过 USB 可写空间。块和整文件 CRC32 均采用 IEEE 多项式 `0xedb88320`、初始值 `0xffffffff`、末尾异或 `0xffffffff`，与 Python `zlib.crc32` 相同，在线为八位十六进制。READ 必须从偏移 0 顺序读取，EOF 的 CRC 覆盖完整文件，包括空文件 CRC 0。

电脑一次仅发一条请求；超时或块校验失败时，最多尝试三次相同 ID/内容。飞控缓存最近一次原始命令和回复，重复 NEXT 不推进目录、重复 READ 不累计两次 CRC；客户端忽略迟到旧 ID。空闲 15 秒、USB 断开或解锁都会释放会话。电脑窗口空闲每三秒发 KEEP。错误仍为 `ERROR code message`，包含 `LOCK_REQUIRED`、`NO_SD`、`PATH`、`OPEN_FAILED`、`SESSION_EXPIRED`、`OFFSET`、`FILE_CHANGED`、`READ_FAILED` 和 `NAME_TOO_LONG`。

文件服务不写 SD 文件；首次会话开始时刷新已打开的日志，然后暂停日志与普通 SD 参数延迟保存。RC_SAVE 在文件会话期间拒绝保存；有待写 SD 参数时重启请求也不会抢占会话进行保存。关闭会话后普通保存流程继续。电脑侧完整校验通过后才将临时文件改成最终名称。

## 构建与验证

在本目录执行 `./build.ps1`，生成 `dist/CoFlyGroundStation.exe`。已通过的自动测试使用本机 socket 模拟飞控，覆盖 V8 文本文件与 CRC、USB 参数导入和失败处理。按项目要求，测试脚本在验证后删除。[旧版板上记录](BOARD_TEST_REPORT.md) 不代表本版已完成真机验证。

## 遥控器整组校准保存

`CAPS` 增加 `radio_calibration=SUPPORTED`，`DATA` 增加 `radio_valid=0/1`（主通道近期有效接收，包含 SBUS 失控/丢帧检查）。九步采样在地面站完成，确认前不改动飞控参数。

命令：`GCS RC_SAVE id min1 max1 trim1 rev1 min2 max2 trim2 rev2 min3 max3 trim3 rev3 min4 max4 trim4 rev4`。严格要求 16 个数值，分别绑定 CH1–4 的 MIN/MAX/TRIM/REV，范围以注册表为准；每轴总行程至少 400 μs，中位与端点距离必须大于现有死区。保留 DZ 和辅助通道。必须锁定且接收机有效，整组验证后同步写入一次 SD 快照，读回校验成功才更新 RAM。

成功：`@COFLY\tid\tRC_SAVE\tstatus=SAVED`，不使用普通逐项 SET 的 PENDING 状态。失败：`ERROR / LOCK_REQUIRED`、`RADIO_INVALID` 或 `SAVE_FAILED`（参数无效或 SD 保存失败）。失败不应用新的遥控 RAM 参数。保存请求超时为 10 秒；超时不能推断板上保存失败，应重新读取核对。该操作同时持久化快照中已有的其他待保存 SD 参数。
