# 软件验证记录（2026-10-22）

## 2026-10-23 main 双模式集成

- main Debug编译、链接通过：FLASH 93,352 B，RAM 31,568 B。
- main 原PID/重力脚本通过；扩展测试覆盖按实物ID映射、两组CAN、第三电机掉线不阻断折叠、循迹故障立即清零、双向故障锁存及程序启停。
- 四项CTest全部通过：`tracking_behavior`、`imu_support_behavior`、`mode_integration_behavior`、`gravity_compensation_behavior`。
- 此次仅修正旧循迹测试中的固定越界输入和yaw方向换算，使其使用当前配置；未为通过测试更改控制逻辑或已调参数。下方目录重组阶段的旧失败记录保留为历史。
- 模式分发测试隔离循迹计算；真实循迹状态机另由独立行为测试验证。未对集成后的main烧录或采集实机曲线；初始化与备份信息见main接入说明。

## 2026-10-23 目录重组复核

- 循迹控制、轨迹生成与 IMU 按应用子目录迁移；19 个源码/头文件在归一化 include 路径和换行后与迁移前指纹一致，逻辑、参数、轨迹表及注释未改。
- 当前 Debug 固件编译、链接通过：FLASH 86,320 B，RAM 23,744 B；SP 库、Drivers、外设初始化未改，`freertos.c` 只更新 IMU 头文件路径。
- `imu_support_behavior` 在 MSVC 19.51 主机工具链下通过，覆盖 SPI 状态、数据换算和周期适配。
- 原有 `tracking_behavior` 未通过：第133行通用故障断言失败后达到30秒测试超时。故障注入用例第153行将 `roll=1 rad` 当作越界，但当前 `kMaxRoll=3.5 rad`；第154行的 `1.1 rad` 也未超过当前 `kMaxLevelTravel=2.5 rad`。这些固定测试值需按当前配置调整，不能把下方旧版全通过记录视为当前参数的测试结果。
- 本次不修改这些测试假设或实机控制参数；未烧录，也未将模块移植到 main。当前固定绘图通道与旧采集脚本的页映射差异见使用说明第7节。

下方为原始版本的历史记录，不表示当前已调参数版本的全部测试通过。

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
