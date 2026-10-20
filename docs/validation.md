# 软件验证记录

日期：2026-10-20（Asia/Shanghai）。
基线：`fc4d964d67fe79840ae3f027f09168f7d2ac0b17`。
分支：`feat/trajectory-tracking`。

## 固件

STM32CubeCLT GNU Arm Embedded GCC 14.3.1，C++17，Ninja。

| 构建 | 结果 | FLASH | RAM |
| --- | --- | --- | --- |
| Debug，`cmake --preset Debug` / `cmake --build --preset Debug` | 完整编译、链接通过 | 83,272 B | 23,656 B |
| Release，`cmake --preset Release` / `cmake --build --preset Release` | 完整编译、链接通过 | 56,808 B | 23,656 B |

两个构建均未输出编译警告。CCMRAM使用为0。
编译通过不等于实际任务运行时间和栈余量已达标。
初次在受限执行环境启动工具链遇到系统DLL重定位错误，
在正常执行环境使用同一STM32工具链完成上述构建。

## 主机行为测试

使用便携w64devkit v2.10.1的桌面GCC 16.2.0，
`-Wall -Wextra -Werror`，保持assert启用。
直接编译实际的 `tracking_controller.cpp`、`tracking_mode.cpp`、
`trajectory_task.cpp`；仅FreeRTOS计时/临界区接口使用单线程测试替身。

`ctest --test-dir build/host --output-on-failure -V`：

```text
PASS controller: feedforward, limits, anti-windup, invalid input, approach
PASS trajectory table: period=680 ms, peak speeds=7.148648 / 8.308258 rad/s
PASS alignment, playback handshake, snapshots and RMS
PASS RTOS tick rollover and playback
PASS faults: remote, motor, timing, stale reference, limits, NaN, alignment, error
All tracking tests passed. Hardware dynamics and ISR scheduling are not simulated.
100% tests passed, 0 tests failed out of 1
```

此外，逐字比较了附件与集成版本的 `kTrajectory` 表，除换行格式标准化外完全一致。
`git diff --check`通过。

## 尚需硬件验证

- A/B正方向、pitch水平零点、机械行程与负载。
- CAN电流控制模式、真实电机反馈新鲜度、UART上位机解析。
- 控制周期、任务栈高水位、真实中断调度和通信故障行为。
- 实测RMS、最大误差、超调、饱和比例与温升。

当前未烧录硬件；没有把理想反馈测试结果当作实机跟踪性能。
