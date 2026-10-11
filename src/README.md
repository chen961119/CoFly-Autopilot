# 源码目录

项目头文件与实现均放在 `src`，同一模块的 `.h`、`.cpp` 存在同一目录。`main.cpp` 保留在根目录，仅编排初始化和主循环。

| 目录 | 职责 |
| --- | --- |
| `config` | 固件配置与运行时飞机身份 |
| `control` | 控制模式、混控、控制分配及运行状态 |
| `sensors` | 传感器采样、校准、滤波与姿态估计 |
| `actuators` | 执行器输出与 PWM 限制 |
| `interaircraft_comm` | 机间命令、状态接收与发送、级联转发 |
| `ground_station_service` | USB 地面站设备服务与数据接口 |
| `radio_comm` | 遥控接收 |
| `telemetry` | 遥测输出 |
| `system` | 时钟、飞行锁定与设备重启 |
| `diagnostics` | 诊断输出与 Teensy 固件调试 |
| `interface` | 按键与 OLED 人机界面 |
| `logging` | SD 飞行日志 |
| `math` | 数学辅助函数 |
| `parameters` | 参数注册、默认值、编解码、存储与 USB 参数服务 |

新增代码放入对应模块，需要新职责时建立新的模块目录；项目不再建立 `include` 文件夹。项目头文件引用统一使用相对 `src` 的路径，例如：

```cpp
#include "config/flight_config.h"
#include "control/control_state.h"
#include "parameters/parameter_service.h"
```

PlatformIO 的 `include_dir = src` 与 `-I src` 提供统一头文件根目录，`build_src_filter = +<*>` 编译所有模块下的源码。普通和调试环境使用相同布局。调试脚本中的文件路径及主机测试中的头文件搜索路径须随文件迁移同步更新。
