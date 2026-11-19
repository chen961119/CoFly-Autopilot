# FreeRTOS Teensy 移植来源

CoFly 使用 Timo Sandmann 维护的 Teensy 4.0/4.1 社区移植，固定在 `v11.2.0_v4` 标签，对应提交 `b1943409de50e0514469ae0fc69a6f9ae17f6118`。库清单的版本为 `11.2.0-4`，其中的 FreeRTOS Kernel 版本为 `V11.2.0`。[上游固定提交](https://github.com/tsandmann/freertos-teensy/tree/b1943409de50e0514469ae0fc69a6f9ae17f6118)

本地保留 `src/` 和 `lib/` 中的构建源码与随附说明，以及上游的 `docs/`、`README.md`、`library.json` 和 `library.properties`。示例与开发工具（包括 `src/portable/.clang-format`）可从固定提交获取。PlatformIO 使用 `library.json` 编译内核、Teensy 适配以及 `lib/cpp` 和 `lib/gcc` 的 C++ 支持源码。

从上述提交移入时，对上游文件的唯一修改位于 `library.json`。头文件搜索路径由 `-I../../../../src` 改为 `-I${PROJECT_SRC_DIR}`，使库移入项目的 `lib/` 后仍能找到项目级配置头。

## 许可索引

上游清单的 `license` 字段写作 `MIT`。打包时请一并保留下列组件各自的文件头、原作者声明与许可文本。

| 文件或组件 | 文件声明的许可 |
| --- | --- |
| FreeRTOS 内核源码、`src/FreeRTOSConfig.h`、`src/freertos_tasks_c_additions.h`、`src/portable/port.c` 和 `portmacro.h` | [MIT](docs/freertos/LICENSE.md) |
| `src/arduino_freertos.h` | [GPL-3.0-only](LICENSES/GPL-3.0.txt) |
| `src/portable/event_responder_support.{h,cpp}`、`runtime_support.cpp`、`teensy.h`、`teensy_4.cpp`、`teensy_common.cpp` | [LGPL-2.1-or-later](LICENSES/LGPL-2.1.txt) |
| `lib/cpp/src/` 和 `src/critical_section.h` | Piotr Grygorczuk 的[三条款 BSD 许可](lib/cpp/LICENSE.txt) |
| `lib/gcc/{condition_variable,future,mutex}.cc` | GPL-3.0-or-later，附 GCC Runtime Library Exception 3.1。[原声明](lib/gcc/LICENSE.txt) · [GPLv3](LICENSES/GPL-3.0.txt) · [例外全文](LICENSES/GCC-exception-3.1.txt) |

`docs/cpp/` 随上游文档一同保留，当中另附 [Code Project Open License](docs/cpp/LICENSE.html)。`LICENSES/` 中三份完整文本逐字复制自 GCC 13.2.0 的发行源码。[COPYING3](https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/COPYING3) · [COPYING.LIB](https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/COPYING.LIB) · [COPYING.RUNTIME](https://github.com/gcc-mirror/gcc/blob/releases/gcc-13.2.0/COPYING.RUNTIME)
