# Tracking Score

`tracking_score.cpp` and `tracking_score.hpp` are copied without algorithm changes from:

https://github.com/you1369-wu/rm_yuntai/tree/d17795fd1a1d18f8d8a9b8586250c5984c02784b/applications

Source branch: `feat/trajectory-tracking`. The source `main` has no scoring module.
Yaw tolerance is strictly less than `0.5f / 57.3f` radians; pitch tolerance is
strictly less than `1.5f / 57.3f`. Both filters use alpha `0.001f` per 1 ms call.

## CubeMX / CMSIS-RTOS v1

Create exactly one task under FreeRTOS > Tasks and Queues:

| Setting | Value |
| --- | --- |
| Task Name | `scoreTask` |
| Priority | `osPriorityAboveNormal` |
| Stack Size (Words) | `256` (1024 bytes on this STM32F4 port) |
| Entry Function | `tracking_score_task` |
| Code Generation Option | `As external` |
| Parameter | `NULL` |
| Allocation | `Dynamic` |

Keep `configTICK_RATE_HZ=1000`, `INCLUDE_vTaskDelay=1` and preemption enabled.
They are already enabled in this project. Keep control at Realtime, IMU at High,
and plot at Low. Measure task stack high-water marks after creating the new task.

The expected generated code is:

```c
extern void tracking_score_task(void const * argument);
/* Inside MX_FREERTOS_Init(): */
osThreadDef(scoreTask, tracking_score_task, osPriorityAboveNormal, 0, 256);
scoreTaskHandle = osThreadCreate(osThread(scoreTask), NULL);
```

Alternatively, the C-compatible declaration is available from
`tracking_mode/tracking_score/tracking_score_task.h`. Do not generate a second
function body for the same entry. The external function already contains an
infinite loop and an `osDelay(1)` after each update. Do not call the two detection
functions again from control_task or plot_task.

`osDelay` is a relative delay, not a fixed-period deadline. Execution and
scheduling can affect the actual sample interval; measure it on hardware when
checking the organizers' 1 ms requirement. `INCLUDE_vTaskDelayUntil` is no longer
required by the scoring or trajectory task. The trajectory phase still uses
actual elapsed ticks, rather than counting loop iterations.

This change does not edit `freertos.c`, the `.ioc`, any driver, or SP middleware.
Until the task is created, both plotted ratios remain zero.

## Data And Lifecycle

- Only `tracking::State::Running` is scored, after leveling and initial alignment.
- One coherent control snapshot supplies actual and target angles in radians.
- Yaw scoring uses IMU yaw relative to the IMU zero recorded at the existing
  leveling calibration. The +/-pi wrap is removed relative to that zero.
  Encoder yaw remains the feedback for motor control, unchanged.
- `tracking_config::kImuYawDirection` defaults to +1. Verify that positive IMU yaw
  agrees with positive trajectory yaw on the installed IMU; change only this
  scoring direction if necessary. It is not necessarily the encoder direction.
- Pitch scoring uses the existing corrected IMU pitch relative to its calibrated
  pitch zero, matching the relative target pitch.
- A fresh Running session resets ratios to zero. Trajectory phase wrap does not.
- Duplicate/stale/non-finite snapshots are skipped. No catch-up samples are made
  after a scheduling overrun. Exiting Running retains the last result for display.
- The fold arm control is unchanged; the scoring task sends no motor commands.

`tracking_score::read_score()` returns a coherent pair of ratios, accepted sample
count, last sample timestamp, and an `active` flag. All state has a single writer.

## Plot

The existing USART1 plot_task sends 7 channels at `tracking_config::kPlotPeriodMs`:

| Channel | Value |
| --- | --- |
| 1 | `yaw_ratio`, in [0,1] |
| 2 | `pitch_ratio`, in [0,1] |
| 3 | Target yaw, rad |
| 4 | Encoder-measured relative yaw, rad |
| 5 | Target small-pitch angle, rad |
| 6 | IMU-measured relative pitch, rad |
| 7 | `distance`, cm; 65535 means no valid distance |

The yaw tracking curve uses encoder feedback, while yaw scoring uses the
calibrated IMU-relative yaw described above. CH1/CH2 are the organizer-defined
filtered ratios, not final points out of 40.
The binary Plotter framing is unchanged, with the new payload length:
`AA BB 1C` then 7 float32 values. This is not text. Configure the receiving plot
for 7 channels instead of the previous 18; distance has a different scale from
angles and ratios.

The ratio is an exponential moving average, not the final score or the full-run
hit-count fraction. The organizers average the displayed ratios over their test
window and calculate `(mean(yaw_ratio) + mean(pitch_ratio)) * 20` out of 40.
