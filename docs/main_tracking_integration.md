# main 双模式接入（2026-10-23）

## 模块和电机

`applications/fold_mode/` 保留 main 原有 PID、参数、重力补偿、标定和折叠流程。
`applications/tracking_mode/tracking_control/`、`tracking_generation/` 与 `applications/imu/` 从已实机测试的循迹工作区按原样导入；两模式不共用 PID 对象或积分。

| 实物 | main 名称 | 折叠职责 | 循迹名称/职责 |
| --- | --- | --- | --- |
| GM6020 ID 1 | motor_b | 保持 yaw | A，yaw 轨迹 |
| GM6020 ID 2 | motor_a | 折叠机构 | B，保持进入位置 |
| M2006 ID 3，36:1 | motor_c | 零力矩，不运行位置保持 | C，IMU 调平和 pitch 轨迹 |

反馈和力矩在 `control_task.cpp` 按上表双向映射。折叠的重力补偿只在折叠模块中计算，不能在公共发送处再次叠加到循迹输出。
唯一发送者仍是 `control_task`：GM6020 共用0x1FE，M2006使用0x200第3槽；每个发送ID每周期只发一帧，其他槽清零。接收回调也只有一处。

## 模式和故障

- 右下：三电机零力矩，保留原折叠开机标定，同时解锁/确认循迹故障。
- 左下且右中/右上：执行自动任务，原折叠模式运行20秒后切换循迹；详见[自动任务说明](mission.md)。
- 右中且左中/左上：无程序循迹请求时执行原折叠逻辑；不依赖ID3或IMU在线。
- 双上：进入循迹；仍需先经过在线右下档，随后按原流程等待IMU、调平、靠近轨迹首点和跟踪。
- `gimbal_tracking_request_start()` 可在手动且非右下时请求循迹，优先于右中折叠；自动期间不接受/排队该请求。`stop()` 对两种控制来源均有效，锁存失能后须回右下才可重启。实现保留在公共控制任务，声明在 `tracking_request.h`。
- 任一模式的故障都阻止进入运动模式，切换到另一模式不能绕过故障；右下同时确认两边故障。
- 模式退出清理对应控制器历史。公共清零不额外发送CAN，周期末统一发送；不保证切换时插入一个独立的零力矩CAN周期。

观察 `control_mode` 判断顶层模式：DISABLED=0、FOLD=1、TRACKING=2。
`control_source` 区分手动/自动，`mission_status` 观察自动阶段、故障和20秒计时。
循迹内部状态和原因仍看 `debug_tracking_state`、`debug_tracking_fault`；折叠仍看 `fold_stage`、`fold_fault`。
为保留循迹调试命名，`debug_motor_a_*` 在此仍指循迹A（ID1，即 main 的 motor_b），不是 main 的折叠 motor_a。

## 任务、配置和绘图

- 原 main 的 IMU、控制、测距、遥控、绘图、LED和蜂鸣器任务只保留各一份；原优先级、任务栈、总堆不变。
- 新增 `trajectory_task` 静态任务：原生优先级4，256个32位word栈（1024字节），不从RTOS heap分配。
- `Core/Inc/FreeRTOSConfig.h` 仅将 `INCLUDE_vTaskDelayUntil` 从0开启为1；轨迹生成依赖此API。再次使用CubeMX生成时须保留此设置。
- `freertos.c` 的轨迹任务和包装入口位于 USER CODE 区；不要在CubeMX另建同名任务。
- `.ioc`、GPIO/SPI/UART/CAN外设初始化、Drivers和SP库均未修改。
- USART1仍只有一个 `plot_task`：循迹及循迹故障输出原18通道，其他状态保留 main 原6通道速度环输出，故障右下确认后恢复原页。帧长度会随模式变化，接收端须匹配。
- 当前旧6通道实际来自 main 的 motor_b（ID1）；沿用原实现，不把已有变量名或旧“A折叠电机”注释当作物理映射。

## 验证与备份

- Debug固件编译通过；折叠PID、重力、模式路由、ID映射、第三电机失联隔离、程序启停及故障锁存测试通过。
- 真实循迹、IMU适配、模式集成、重力补偿四项CTest通过。旧循迹测试仅将写死的越界值/转向改为当前配置，运行参数没有回退。
- 主机模式测试以循迹桩隔离分发器；真实循迹状态机由独立测试编译真实代码验证。主机测试不能代替本次合并后的实机复测。
- 未烧录、提交或推送。原分支未提交内容完整保留在stash提交 `0d98f5f1dcea9df339104521bf9e51fbbe44b061`，包含未跟踪文件，未删除备份。
- 恢复原分支时先妥善保存main工作区，再切回 `feat/trajectory-tracking` 并应用该stash；不要直接在已集成的main上应用整份备份。
