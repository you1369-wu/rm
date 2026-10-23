# 三电机云台：IMU 调平后轨迹跟踪

本版本按 2026-10-22 的最终需求重写控制流程：A=GM6020 ID1（yaw），B=GM6020 ID2（大 pitch 保持），C=M2006 ID3（36:1 减速箱，小 pitch）；C 板 IMU 随 C 电机末端运动。

完整流程：保持 B 进入时的位置 → 等待 IMU 稳定 → C 调平到 pitch≈0 → 标定轨迹零点 → 平滑靠近轨迹首点 → A/C 循环跟踪附件轨迹。

- [操作、参数、前一阶段接入、绘图通道与调试说明](docs/trajectory_tracking.md)
- [软件验证记录及实机边界](docs/validation.md)
- 集中参数：`applications/tracking_config.hpp`
- 主控制状态机：`applications/tracking_mode.cpp`
- 前一阶段启动接口：`applications/tracking_request.h`
- 串口采集及绘图：`tools/capture_tracking.py`、`tools/plot_tracking.py`

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

源码包包含 HAL、FreeRTOS 和 sp_middleware，不需要在线拉取子模块。使用已有 STM32CubeCLT 的 ARM GCC、CMake、Ninja，确保三者在 PATH。旧两轴逻辑和旧说明中的“B 跟踪 pitch / 固定 0.2 rad”均不适用于本版本。

已完成本地编译和主机行为测试，尚未连接、烧录或调试你的实物。首次使用先按说明核对 IMU 安装零偏、C 电机方向及机械行程，再调整 PID/前馈。
