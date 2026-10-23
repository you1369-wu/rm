# IMU 调平与轨迹跟踪使用说明

## 1. 本次实现的任务与坐标

本工程保留你提供的 STM32F407 C 板底层配置、遥控器、CAN、BMI088、Mahony 和绘图中间件；重写原有轨迹控制状态机。附件中 681 个采样点和缩放/插值规则保持不变，默认周期 680 ms。

| 电机 | 型号/ID | 本版本职责 | CAN反馈/控制帧 |
| --- | --- | --- | --- |
| A | GM6020 / 1 | yaw 轨迹跟踪 | 0x205 / 0x1FE 的第1槽 |
| B | GM6020 / 2 | 闭环保持进入跟踪时的编码器位置 | 0x206 / 0x1FE 的第2槽 |
| C | M2006 / 3，36:1 | IMU调平和末端pitch轨迹跟踪 | 0x203 / 0x200 的第3槽 |

三电机接 CAN1。GM6020 沿用所给中间件的**电流控制模式**，实际电调必须匹配；不能将这些 N·m 指令直接改用电压模式。CAN 分两帧发送，未使用槽位清零，唯一发送者为 `control_task`。

C 的编码器角度、转速和力矩经 `RM_Motor(..., M2006, 36, true)` 换算到减速箱输出轴；上层不再除以/乘以36。C闭环角度与速度取末端IMU，C编码器用于等待阶段保持和机械行程约束。A 使用连续编码器，yaw 零点为调平完成时 A 的位置；不使用易漂移的 IMU yaw。B 的保持点只在每次进入时捕获一次。

pitch 原点是“IMU实际接近水平且静止”时的姿态，不是把任意起始倾角直接清零。标定后保存 IMU 残余倾角、C编码器原点及 A编码器原点；IMU解算器本身不清零。绘图 pitch 实际值为 `校正后的IMU pitch - 标定残余倾角`。此方案假设基座基本固定、两级pitch轴平行、IMU随末端转动，且安装后 euler.pitch_rad 对应末端俯仰。你已确认 IMU 随 C 末端转动。

## 2. 操作与自动衔接

手动启动：遥控器在线，右拨杆先置下解除锁定，再将左右拨杆均置上。右拨杆置下会立即取消跟踪并发送零力矩；故障后也须右下确认。默认其他档位不执行原有旧循迹动作。

程序衔接：前一阶段完成且停止发送电机指令后，在任务上下文调用：

```cpp
#include "tracking_request.h"

// 在“前一阶段刚完成”的事件中调用一次，不能在每个周期重复抢占控制。
gimbal_tracking_request_start();

// 需要停止时调用；停止锁存后需右下档重新解锁。
gimbal_tracking_request_stop();
```

两个函数是独立的事件接口，示例中的开始和停止不能紧挨着同时调用。启动前须有一次遥控在线的右下档解锁，调用启动时右拨杆须已离开下档。软件请求允许在右中档接入，不要求双上；右下、遥控失联或 stop 始终优先。遥控仍须在线。函数只提交请求，控制由 `control_task` 完成；不可在中断调用。

交付工程不包含你未提供的“前一阶段完成”业务事件；因此提供可独立运行的双上启动方式与上述真实接口。若将前一阶段重新接回本工程，应由同一控制任务管理前后阶段的指令所有权，不能再增加另一个 CAN 发送任务。B 将保持接口生效那个控制周期测到的位置。

## 3. 状态流程

| 状态值 | 名称 | 行为及转移 |
| --- | --- | --- |
| 0 | Disabled | 零力矩，等待启动 |
| 1 | WaitImu | A/B/C先保持进入位置；连续500 ms传感器新鲜、各轴接近静止后进入调平；5 s超时 |
| 2 | Level | B保持，A保持；C以IMU pitch闭环，用2 s五次曲线接近水平，最大目标速度0.5 rad/s |
| 3 | Align | 调平确认后标定；用2 s五次曲线从轨迹零点移动到附件首点 |
| 4 | WaitTrajectory | 请求轨迹任务从相位0启动，等一致性快照确认 |
| 5 | Running | 循环执行680 ms轨迹；B持续保持原位置；统计跟踪误差 |
| 6 | Fault | 故障锁存，全部输出零；右下确认后可重新调平 |

调平完成条件：IMU pitch绝对值<0.01 rad（约0.57°）、末端pitch速度及电机速度小于阈值、A/B位置误差足够小，持续300 ms。不是刚经过0就标定。调平总超时10 s。对齐轨迹首点后，误差<0.03 rad且速度<0.08 rad/s持续300 ms才启动，超时8 s。

**附件首点不是(0,0)**：yaw=0.314159 rad（18°），pitch=-0.523599 rad（-30°）。水平点是坐标原点，从该点平滑到首点，再播放原始表。没有平移或重造比赛轨迹。轨迹范围为yaw±22.5°、pitch±30°；峰值速度约7.15/8.31 rad/s。

每次退出再进入都会重新调平、标定并从轨迹开头播放。B的保持积分在调平/对齐/播放切换时保留，避免每次切换失去重力负载所需力矩；退出和故障时清除。

## 4. 双环与PDF前馈

各轴为角度P外环、速度PI内环，速度前馈与力矩前馈分别配置：

```text
error_q = q_ref - q_actual
omega_set = clamp(Kp_angle*error_q + Kvff*trajectory_velocity)
alpha_raw = (omega_set[k] - omega_set[k-1]) / dt
alpha_set = 一阶低通(clamp(alpha_raw))
tau_ff = J*alpha_set + b*omega_set + Kg*cos(q_physical)
tau_pid = Kp_speed*(omega_set-omega_actual) + I
tau_cmd = clamp(tau_pid + tau_ff)
```

这落实了PDF要求的“速度环目标 → 加速度/阻尼前馈 → 与速度PI输出相加 → 总力矩限幅”。加速度来自**限幅后的速度目标变化率**，而非实测角速度差分，也不把轨迹表的加速度再重复相加。首个周期不做速度差分，切换控制器时清除差分记忆。轨迹表仍完整保留原始加速度，可在调试器读取。

积分按秒积分，并针对含前馈的最终力矩做条件抗饱和。惯性、阻尼和重力系数默认0，速度前馈默认1（B为0）。电机型号和ID不足以确定云台负载惯量；因此交付的是已实现的可整定前馈通路，不虚构实机J/b值。C的J/b/力矩均应按减速箱输出端等效，A/B按直接驱动轴。

IMU角速度取Euler pitch变化率 `wy*cos(roll)-wz*sin(roll)`，不直接把gyro.y当作所有姿态下的pitch速度。IMU数据在短临界区内一次发布/读取，带时间戳；读取或调度异常不会刷新为一份“新的有效数据”。BMI088 SPI失败通过新增状态接口上报；Mahony按实际采样间隔积分。

## 5. 实物参数核对和整定

集中修改 `applications/tracking_config.hpp`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| kTipRatio | 36 | 已按你确认的原装减速箱设置 |
| kTipDirection | +1 | 正输出力矩应使校正后的IMU pitch增加；反向安装改为-1 |
| kImuPitchDirection | +1 | IMU pitch坐标定义；与电机方向分开设置 |
| kImuPitchMountOffset | 0 rad | `校正pitch=方向*原始pitch-安装偏置`，有安装倾斜时需实测 |
| kYawDirection / kArmDirection | +1 / +1 | 编码器与输出力矩一同变换 |
| yaw范围 | [-0.50,+0.50] rad | 相对标定yaw零点 |
| pitch范围 | [-0.60,+0.60] rad | 相对标定pitch零点 |
| C范围 | [-0.70,+0.70] rad | 相对调平时的C编码器位置 |
| kMaxLevelTravel | 1.0 rad | 标定前C相对进入位置的最大移动量 |
| A/B/C力矩上限 | 0.30/0.30/0.20 N·m | 仅初始软件限制，不代表足够克服你的负载 |
| A/B/C速度Kp | 0.035/0.20/0.20 | 需要实机整定 |
| A/B/C速度Ki | 0/0.08/0 | B有保持积分，其余初始关闭 |
| 加速度滤波时间常数 | 0.010 s | J前馈微分支路 |

建议先确认C正负方向与安装偏置，再调B静态保持，之后调C低速调平、A/C角度跟随，最后在同一轨迹下比较速度前馈和J/b前馈。若B持续偏移或输出饱和，需要检查负载与力矩上限，不能把“保持目标不变”当作机械保证。

软件限位是相对于进入/标定位置的范围，不能识别断电前的绝对机械位置。必须保证初始姿态以及“调平+±30°轨迹”都位于实际机械行程内。默认允许roll约±20°、绝对pitch约±68.8°，用于排除不适合当前Euler pitch控制的安装/姿态。

故障/停止时全部电机零力矩，B也会卸力；这不是带负载的机械制动。评估实际稳定性还需观察振荡、超调、温升、通信和控制周期，当前没有实测曲线或调好的参数。

## 6. FreeRTOS、构建和文件

- `trajectory_task`：独立静态任务，1 ms更新，优先级4，256 word栈。
- `control_task`：1 ms，原有优先级6、512 word栈，唯一电机输出。
- `imu_task`：1 ms绝对周期调度，按实际间隔更新Mahony，原有512 word栈。
- `plot_task`：10 ms，优先级1，512 word栈；使用阻塞串口发送，避免DMA复用发送缓冲。
- tick须为1000 Hz、32位；源码有静态断言。1 ms为设计周期，尚需实测最坏执行时间与栈高水位。

新增任务保留在 `Core/Src/freertos.c` USER CODE 区，CMake源文件已注册，`INCLUDE_vTaskDelayUntil=1` 已开启。不要再通过CubeMX创建同名任务。`usart_task`保留入口但不发送文本，以免与绘图二进制帧混流。

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

主要实现文件：`tracking_mode.cpp/.hpp`、`tracking_controller.cpp/.hpp`、`tracking_config.hpp`、`tracking_request.h`、`control_task.cpp`、`imu_task.cpp/.h`、`can_task.cpp`、`tools/plot_task.cpp`、`trajectory_task.cpp/.hpp`、`trajectory_runtime.hpp`。

中间件修改：BMI088新增SPI传输成功查询、失败不发布有效IMU；Mahony支持实际dt、pitch初始化公式修正和asin输入限幅。请使用完整包或同时更新这些文件，不能只替换控制任务。

## 7. 绘图输出与实测数据

USART1，115200、8N1，原工程PB6 TX/PB7 RX。串口数据为 `AA BB 48` + 18个小端float32，共75字节，无帧尾、无CRC。100 Hz约7500 B/s。不是普通文本/CSV，也不是默认JustFloat；用匹配解析器。

默认 `kPlotPage=0`：

| 通道 | 字段 | 单位 |
| --- | --- | --- |
| 1/2 | yaw目标/实际 | rad |
| 3/4 | 末端pitch目标/实际 | rad |
| 5/6 | yaw/pitch误差 | rad |
| 7 | B保持误差 | rad |
| 8 | C相对调平原点的编码器角 | rad |
| 9/10/11 | A/B/C最终给定力矩 | N·m |
| 12/13 | yaw/pitch累计RMS（控制侧统计） | rad |
| 14/15 | 状态/故障码 | 整数 |
| 16 | 轨迹相位 | ms |
| 17 | 饱和掩码：A=1，B=2，C=4 | 位掩码 |
| 18 | 校正后绝对IMU pitch，未减标定残差 | rad |

`kPlotPage=1` 编译后为诊断页：1/2 yaw总速度目标/实际；3/4 pitch总速度目标/实际；5/6/7 A反馈力矩/前馈力矩/最终力矩；8/9/10 C对应力矩；11/12 A/C滤波目标加速度；13 B力矩；14 B误差；15状态；16故障；17控制间隔ms；18饱和掩码。不能用page0解析器解释page1。

`tracking::read_telemetry()`还包含控制侧峰值误差、样本数、饱和样本数、时间戳和全部标定原点。饱和比例=`saturated_samples/samples`。RMS只在状态5累积，每次启动清零；退出后保留最后统计用于查看。等待IMU时尚未启用pitch姿态环，该阶段图上的pitch目标随反馈显示，不用于跟踪评分。

采集并绘图（在工程根目录，端口按实际替换）：

```powershell
python -m pip install pyserial matplotlib
python tools/capture_tracking.py --port COM5 --seconds 30 --page 0 --csv logs/run.csv
python tools/plot_tracking.py logs/run.csv --output logs/report
```

生成原始CSV、`tracking.png`和`metrics.json`；后者按每个连续Running区间给出RMS、峰值、平均误差、B偏差和饱和比例。仅解析已有二进制记录时可使用 `--input-bin capture.bin` 代替 `--port`。

主机采集是100 Hz抽样，控制侧RMS/峰值是1 kHz控制循环统计；不要把抽样峰值当作所有尖峰。离线二进制流不含时间戳，绘图横轴只能按100 Hz无丢帧估计；串口在线采集记录主机接收时间，有缓存抖动。协议无CRC，采集脚本能处理分包/噪声找帧，但不能证明每帧无传输错误。

## 8. 故障码

| 值 | 含义 |
| --- | --- |
| 0 | 无故障 |
| 1 | 尚未右下解锁 |
| 2 | 任一电机掉线（沿用中间件100 ms阈值） |
| 3 | 运行中IMU未就绪或超过10 ms不新鲜 |
| 4 | 电机/IMU/目标出现NaN或Inf |
| 5 | yaw、pitch或C编码器/目标越过软件行程 |
| 6 | 控制间隔为0或超过5 ms |
| 7 | 调平10 s内未完成 |
| 8 | 对齐首点8 s内未完成 |
| 9 | 轨迹未就绪、过期或启动/运行异常 |
| 10 | Running误差>0.35 rad持续200 ms |
| 11 | 活动期间遥控失联 |
| 12 | B保持误差>0.10 rad持续200 ms |
| 13 | 等待IMU稳定5 s超时 |
| 14 | IMU绝对pitch或roll超出姿态工作区 |

切右下清除故障后重新启动，会重新调平，不能直接从旧相位继续。
