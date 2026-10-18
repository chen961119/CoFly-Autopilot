# CoFly Autopilot 文档

CoFly Autopilot（Collaborative Distributed Autopilot，分布式协同飞控系统）的项目介绍和快速开始见 [仓库首页](../README.md)。

## 现行文档

| 文档 | 阅读目的与维护范围 |
| --- | --- |
| [系统架构](architecture.md) | 实现边界、模块职责、主循环及修改入口 |
| [控制架构](control-architecture.md) | 主从流程、目标与输出接口、模式、混控和融合 |
| [俯仰 INDI](indi-variable-naming.md) | 控制公式、PWM 与逻辑输出换算、变量单位和模型限制 |
| [固件指南](firmware-guide.md) | 编译、配置、接线、锁定、方向迁移及日志 |
| [参数与持久存储](parameter-storage.md) | 首次配置、参数注册、SD/EEPROM、V8 格式及迁移 |
| [软件需求](requirements.md) | 唯一需求编号、目标行为及验收矩阵；不代表全部已实现 |

每个主题只在对应文档维护详细说明，其他文档通过链接引用。现行约定以这些文档及实际源码为准；历史方案不作为配置步骤。

## 配套工具

| 入口 | 内容 |
| --- | --- |
| [USB 地面站](../tools/ground_station/README.md) | 参数配置、协议、SD 恢复与电脑端使用 |
| [飞行数据浏览器](../tools/flight_data_browser/README.md) | SD 日志、多机曲线、FFT 和数据导出 |
| [板上调试](../tools/teensy_debug/README.md) | TeensyDebug 环境与 VS Code 调试 |
| [参数板上测试记录](../tools/ground_station/BOARD_TEST_REPORT.md) | 2026-10-07 的测试范围与结果 |

源码维护入口见 [源码目录](../src/README.md)、[参数模块](../src/parameters/README.md) 和 [机间通讯模块](../src/interaircraft_comm/README.md)。

## 研究与历史

- [Zhu 等（2025）参考论文](references/zhu-2025-morphing-control.pdf)：Aerodynamics-Driven Morphing Control and Flight Test for Compound Flexible Multibody Aircraft。
- [研究方法与控制演进](archive/CONTROL_ARCHITECTURE_ANALYSIS.md)：论文与实现的区别、保留的五体分配模型及旧版对照结论。

2026-10-18 整理时，重复专项报告已并入上述主题，过期仓库快照与未实施方案已移除，`validation` 目录中的数值结果、补丁和方法文件已删除。历史软件检查不作为当前工作区或实机验证通过的证明。
