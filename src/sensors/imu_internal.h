#pragma once

// 全局采样与历史值均为 SI：加速度 m/s²，角速度 rad/s。
// 仅供 sensors 模块内部采样和姿态融合使用，业务模块使用统一测量数组。
extern float AccX_6050, AccY_6050, AccZ_6050;
extern float GyroX_6050, GyroY_6050, GyroZ_6050;
extern float Gyro_X_EXT, Gyro_Y_EXT, Gyro_Z_EXT;
extern float Acc_X_EXT, Acc_Y_EXT, Acc_Z_EXT;
