# Yuntai PID Integration

Target: `feature/wuyou-yuntai_fold`, based on `2dfe6c3`.

## Source Mapping

| Target motor | Parameter source | Source commit | Target CAN ID |
| --- | --- | --- | --- |
| A, folding joint | `feature/motor_b_pid` | `f909131` | 1, unchanged |
| B, yaw/hold joint | `feature/pid_motor_a` | `965feed` | 2, unchanged |

The source test motor CAN IDs and source test targets (`angle + 1.3` or
`angle + 2`) are NOT copied. The target branch still controls folding targets
and captures B's hold angle in its original state machine.

## PID Parameters

`dt` is seconds. Angle-loop `mo/mio` are rad/s; speed-loop `mo/mio` are N*m.

| Loop | dt | kp | ki | kd | mo | mio | alpha | angular | dynamic |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A angle | 0.001 | 5.0 | 2.0 | 0 | 5.0 | 2.5 | 1.0 | false | true |
| A speed | 0.001 | 0.35 | 0.04 | 0 | 0.45 | 0.2 | 1.0 | false | true |
| B angle | 0.001 | 5.0 | 2.51 | 0 | 5.0 | 2.5 | 1.0 | true | true |
| B speed | 0.001 | 0.1 | 0.08 | 0 | 0.3 | 0.2 | 1.0 | false | true |

A's angle-loop `angular=false` is intentionally retained from the folding
branch. The tuning source used `true`, but folding must not choose a wrapped
path through a mechanical stop. Other parameters match their mapped sources.

## Gravity Compensation And Startup Reference

Only A receives gravity feedforward: the B source has no gravity model.
The model uses the same tuned equivalent mass, 0.209 kg, and the same CAD
geometry as `feature/motor_b_pid`:

```text
x0 = 0.011656 m
z0 = -0.097403 m
delta = motor_a.angle - motor_a_gravity_horizontal_angle_rad
gravity_torque = 0.209 * 9.81 * (x0*cos(delta) - z0*sin(delta))
given_torque_A = clamp(speed_PID.out + gravity_torque, -0.45, +0.45)
```

The operator confirmed the startup mechanical maximum is also the horizontal
reference pose. The existing startup procedure is retained:

1. Place the joint at the mechanical maximum before enabling power/control.
2. Keep the remote right switch DOWN. With fresh feedback and stationary A for
   100 ms, the existing calibration validates the mechanical offsets and captures
   the reference once for this power-on session.
3. Successful calibration also saves that same angle as the gravity reference.
4. MID/UP keep their original folding behavior. DOWN, faults and lost feedback
   still send zero torque, including disabling gravity feedforward.

The three mechanical offsets remain `NAN`, exactly as in the target branch.
Fill `yuntai_other_limit_offset_rad`, `yuntai_unfolded_offset_rad` and
`yuntai_folded_offset_rad` using measured values before automatic motion.
Until then calibration fails and motion stays disabled. No offsets are guessed
from the single-motor test setpoints.

The original A travel-direction check runs AFTER gravity addition and total
torque limiting. Gravity cannot bypass that check. B still holds the original
captured angle using its own PID. State transitions, hysteresis, timeout,
arrival checks, CAN callback, laser and UART tasks are not replaced.

## CubeMX Differences To Review Manually

No `.ioc`, `Core`, `Drivers`, `Middlewares`, CMake, or `sp_middleware` files
are changed by this integration. Both tuning branches have identical generated
initialization settings. Compared with the folding branch:

| Setting | Folding branch | Tuning branches | Integration guidance |
| --- | --- | --- | --- |
| CAN1 prescaler / SJW | 3 / 1 TQ | 3 / 1 TQ | Unchanged |
| CAN1 BS1 / BS2 | 11 / 2 TQ | 10 / 3 TQ | Optional reproduction of tuning setup |
| CAN1 Auto Bus-Off | Enabled | Disabled | Difference, not required for PID; keep recovery unless deliberately testing otherwise |
| CAN1 Transmit FIFO Priority | Disabled | Enabled | Optional reproduction of tuning setup |
| CAN1 RX0 IRQ | Enabled, priority 5/0 | Same | Keep enabled |
| CAN1 RX1 IRQ | Not enabled | Enabled, priority 5/0 | Optional; current filter routes feedback to FIFO0 |
| USART1 | 115200, 8N1 | Same | No UART parameter change needed |
| USART6 | 115200, 8N1, PG14 TX/PG9 RX | Removed | KEEP for laser distance input |
| USART6 RX DMA | DMA2 Stream1, Channel5, Normal, Low | Removed | KEEP, including byte widths and memory increment |
| USART6 / DMA2 Stream1 IRQ | Enabled, priority 5/0 | Removed | KEEP both enabled |
| FreeRTOS heap | 24576 bytes | 15360 bytes | KEEP 24576 for folding and laser tasks |
| usart_task | Low, stack 512 words | Removed | KEEP existing calibration printing |
| laser_task | Low, stack 512 words | Removed | KEEP; required for folding distance |
| plot_task | Not created | Low, stack 256 words | Only add for plotting; coordinate USART1 ownership |
| led_task / buzzer_task stack | 512 / 512 words | 128 / 128 words | No reduction needed for integration |

CAN remains 1 Mbit/s in both configurations with a 42 MHz CAN clock:
`42 MHz / (3 * (1 + 11 + 2)) = 42 MHz / (3 * (1 + 10 + 3))`.
Matching source CAN options is not a prerequisite for these numerical PID gains.
Do not copy removal of USART6, its DMA/IRQs, or the laser task from test branches.

`control_task` remains Realtime/512 words, IMU High/512, and remote Normal/512.
USART3 remote settings, system clocks and motor CAN IDs are unchanged.
GM6020 current-loop enablement is a motor-side setting, not a CubeMX option.

## Verification

Firmware: `cmake --build build/Debug -j 4`.

Host tests: `powershell -File tests/run_pid_tests.ps1 -Compiler C:/MinGW/bin/g++.exe`.
Fakes replace only HAL/CMSIS scheduling and CAN transport; tests use the actual
control helpers, PID, math and RM_Motor CAN packing implementations. They do
not run motors. Real load stability and safety margins still need bench checks.
