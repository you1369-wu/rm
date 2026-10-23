# 软件验证记录（2026-10-22）

本记录仅对应本次“B保持 + 末端IMU调平 + A/C轨迹跟踪”版本，替代原包中过时的两轴测试说明。

## 固件构建

本机 STM32CubeCLT 1.22.0，ARM GNU GCC 14.3.1，C++17，Ninja。

| 构建 | 结果 | FLASH | RAM |
| --- | --- | --- | --- |
| Debug | 完整编译、链接通过 | 84,672 B | 23,536 B |
| Release | 完整编译、链接通过 | 56,400 B | 23,536 B |

CCMRAM使用为0。最终构建没有编译警告。受限进程环境最初出现Windows系统DLL重定位错误，随后使用本地正常进程环境中的同一工具链完成构建。未连接调试探针或烧录电路板。

## 主机测试

w64devkit GCC 16.2.0，`-Wall -Wextra -Werror`，保持assert启用。测试直接编译真实控制器、状态机、轨迹任务和RM电机中间件，不以另写的控制算法替代被测源码。轨迹任务在线程中运行，测试调度器在每次任务延时边界交替执行轨迹/控制，保持确定性；没有模拟真实中断抢占。

```text
PASS controller: PDF J*d(speed_set)/dt+b*speed_set, reset, limits, antiwindup
PASS actual trajectory: 680 ms, peaks 7.148648/8.308258 rad/s
PASS actual RM middleware: IDs, frame slots, reduction, torque, rollover
PASS full state sequence, B hold, origin, restart, RMS and tick wrap
PASS motor/IMU/remote loss, NaN, attitude/travel, timing, B drift,
     all stage timeouts, stale trajectory, tracking error
All host tests passed. No hardware dynamics, ISR timing or physical stability claim.
100% tests passed, 0 tests failed out of 1
```

测试采用理想运动学反馈跟随目标来验证状态切换与数据流，不包含电机惯量、摩擦、重力、减速箱间隙或真实IMU噪声。测试中的低误差不是实机成绩，也不能证明PID参数稳定。

复现（桌面C++编译器，不是ARM交叉编译器）：

```powershell
cmake -S tests -B build/host -G Ninja '-DCMAKE_CXX_COMPILER=g++'
cmake --build build/host
ctest --test-dir build/host --output-on-failure -V
```

另已验证：

- 集成版本681点轨迹表与用户另附的trajectory_task压缩包逐字一致（换行归一化后）。
- Python脚本语法检查通过；串口解析器通过分包、帧前噪声、非法长度与连续多帧测试。
- 离线误差统计使用已知输入验证RMS、峰值和饱和比例。
- 未采集实际硬件数据，未生成冒充实测结果的曲线。

## 待实机验证的边界

1. C转动方向、IMU安装零偏及pitch轴定义、B保持所需负载力矩、机械可达范围。
2. 初始PID增益、J/b/重力补偿参数。J/b/重力当前为0，需辨识后开启。
3. 控制/IMU实际周期、栈高水位、通信延迟和真实失联行为。
4. 至少10个稳态周期的RMS、峰值误差、超调/相位差、振荡及饱和比例。
5. 原中间件CAN发送接口不返回成功状态，尚无应用层CAN bus-off恢复/发送失败锁存。BMI088新增SPI状态检查，但未实现独立传感器数据就绪/重复样本故障诊断。

B保持是闭环保持目标位置，不是机械锁定；故障或停止时发送零力矩。
