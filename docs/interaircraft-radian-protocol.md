# 机间通信 0.001 弧度协议：修改和验证

日期：2026-10-18。本次继续全局 SI 迁移，将机间通信的线上角度从 0.1 度改为 0.001 rad，角速度从 0.1 度/秒改为 0.001 rad/s。调试打印仍为度、度/秒。

## 编码约定

```text
角度编码   = round(角度(rad) × 1000)
角速度编码 = round(角速度(rad/s) × 1000)
解码       = int16编码值 ÷ 1000
角度打印   = 解码值 × 180/π，单位 deg，保留三位小数
角速度打印 = 解码值 × 180/π，单位 deg/s，保留三位小数
```

正负半步均向远离零的方向取整，理论量化误差上限为半步，另有 float32 舍入误差。int16 可表示 [-32.768, 32.767] rad 或 rad/s。命令目标超范围或为 NaN/Inf 时不更新缓存；状态超范围时饱和，NaN/Inf 编码为 0。异常占位值没有新增有效性标志，不能当作有效测量。

命令最大值检查使用与 float32 解码倒数相同的步长，避免开启 `-ffast-math` 时有效的最大编码回读值因一个浮点舍入位而被拒绝。

## 指定八处逐项修改和验证

| 修改位置 | 具体改法 | 对应验证 |
| --- | --- | --- |
| `CommandPayload` | `pitchAngles` 保持 int16，改为 0.001 rad；在载荷类型内部声明静态 `ANGLE_SCALE=1000` | 实际命令帧仍为 40 字节，载荷 37 字节；三个槽位的全部 65,536 个编码逐个检查；静态成员不占报文字节 |
| `setCommandLink` | 目标 rad 直接乘 1000，正负对称四舍五入；范围检查同步到新量程；五个逻辑输出仍乘 4096 | 每个 int16 端点/中间值、八个半步附近样本、NaN/±Inf/越界拒绝及缓存保持；逻辑输出的布局与定点整数逐一核对 |
| `getCommandLink` | `pitchAngles` 直接除以 1000 发布 rad，去掉度转弧度 | 三个链路层级对应槽位，全编码值往返验证；坏校验与半帧保留上次结果 |
| `AircraftCommandLink::printReceivedData` | 先除 1000 再转度，标签 `pitchAngles (deg)`，三位小数；输出逻辑整数打印不变 | 通过真实接收函数触发打印并捕获字符串；编码 1000 必须打印 `57.296` |
| `SensorPayload` | 五组三槽位字段仍为 int16，角度步长 0.001 rad，`rollRates` 步长 0.001 rad/s；类型内部静态缩放常量 | 实际状态帧仍为 34 字节，载荷 31 字节；字段偏移、左右 bool 与校验均验证 |
| `getSensorLink` | 三轴姿态和 X 轴滚转角速度直接除 1000；Y/Z 角速度不改动 | 左右两条链、三个槽位、全部 65,536 个编码；全局发布值与 rad/rad/s 预期逐项比较 |
| `setSensorLink` | 本机及下游反馈直接使用 rad/rad/s，相邻滚转角差在弧度中计算；五组字段乘 1000；添加饱和及非有限值检查 | B～G 六种身份与三层链路，包含本机、内侧清零、外侧汇总、相邻相对角；超量程和 NaN/Inf；编码偏移和数值逐项核对 |
| `AircraftSensorLink::printReceivedData` | 五组字段均除 1000 后转度；`rollRates` 用 `deg/s`，其余用 `deg`，三位小数 | 捕获实际打印：相对角 `57.296`、角速度 `-28.648`、滚转 `90.012`、俯仰 `-57.296`、偏航 `0.000` |

精确源码行号见文末。

本次修改前后作了范围检查：通信 `.cpp` 只修改以上六个方法体；`.h` 只修改两种载荷定义及 `setCommandLink` 的说明注释。`configure`、收发、转发、时效和限频实现保持原样，符合目录 README 的修改规则。

## 验证执行

```powershell
& C:/Users/chen/.platformio/penv/Scripts/python.exe tools/verification/verify_interaircraft_radians.py
& C:/Users/chen/.platformio/penv/Scripts/python.exe tools/verification/verify_si_units.py
& C:/Users/chen/.platformio/penv/Scripts/python.exe -m platformio run -e teensy41 -e teensy41_debug
git diff --check
```

专项测试编译实际 `interaircraft_comm.cpp`，通过公共收发接口操作真实帧；使用硬件桩而不是复制通信实现。分别使用 `-O2` 与 `-O2 -ffast-math`，每种配置通过 4,456,601 项断言，共 8,913,202 项。

上轮 SI 回归测试的普通控制量容差不变。机间协议对照旧基线时，旧版 0.1 度截断与新版 0.001 rad 四舍五入的理论差异上限为 `0.1 + 0.0005×180/π ≈ 0.128648` 度，因此仅协议对照部分容差同步为 0.129 度，直接的新协议验证仍逐个检查准确的原始整数编码。独立遥测仍用原来的 0.1 度编码。

| 检查 | 结果 |
| --- | --- |
| 新协议专项测试：普通优化 | PASS，4,456,601 项断言 |
| 新协议专项测试：`-ffast-math` | PASS，4,456,601 项断言 |
| 五种机数数值对照 | 各 79,948 个数值通过；最大绝对差 0.11216927，出现在协议量化对照，低于上述 0.129 度上限 |
| 内外置传感器数值对照 | 均通过；随后才执行旧控制器常量审计 |
| 通信修改范围检查 | PASS：仅两种载荷定义、六个指定方法体；接口注释同步 |
| 最终普通固件 `teensy41` | PASS，10.114 s |
| 最终调试固件 `teensy41_debug` | PASS，12.993 s |
| `git diff --check` | PASS |
| 旧全局迁移的控制器常量审计 | 未通过：当前 `roll_pid_controller.cpp` 七处 `phiA`～`phiG` 已改成直接读 rad，原 57.3 换算常量已去除；本次没有修改这个控制器文件，也没有放宽或移除该审计 |

`verify_si_units.py` 完成数值对照后，在上述旧审计处退出失败，因此不将该脚本的整体退出结果描述为通过。已保存的五种构型输出另行按同一容差重新比较，全部通过。本次机间协议专项测试和两套固件构建均独立通过。


## 兼容范围

命令帧仍为 `55 60` / 40 字节，状态帧仍为 `55 71` / 34 字节；线上字段顺序、小端序、校验、槽位和逻辑输出 ×4096 不变。命令级联验证完整帧逐字节相同，不重复量化。时间单位与命令/状态 100/200 Hz 限频不变。

新旧角度编码不兼容，各机必须同步使用新协议固件。本次没有修改控制参数、局部控制状态、独立遥测编码、GCS 或 OLED 显示。验证包含主机模拟与交叉编译，没有烧写设备或进行真实链路/飞行验证。

## 精确位置

| 位置 | 文件 | 起始行 |
| --- | --- | --- |
| CommandPayload | [src/interaircraft_comm/interaircraft_comm.h](../src/interaircraft_comm/interaircraft_comm.h) | 32 |
| SensorPayload | [src/interaircraft_comm/interaircraft_comm.h](../src/interaircraft_comm/interaircraft_comm.h) | 92 |
| setCommandLink | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 31 |
| getCommandLink | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 120 |
| 命令 printReceivedData | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 146 |
| getSensorLink | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 204 |
| 状态 printReceivedData | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 219 |
| setSensorLink | [src/interaircraft_comm/interaircraft_comm.cpp](../src/interaircraft_comm/interaircraft_comm.cpp) | 244 |
