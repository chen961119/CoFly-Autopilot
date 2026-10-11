# EEPROM 主备恢复测试

执行 `./tools/parameter_eeprom_tests/run.ps1`，需要 MinGW g++。

测试直接编译 `src/parameters/parameter_eeprom.cpp`，以模拟 EEPROM 注入每个字节写入位置的断电及写入失败，检查主槽优先、备份恢复、主槽修复、未变化时不写入、两槽失效和容量不足。不代表实板 Flash 仿真层的断电测试。
