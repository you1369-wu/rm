#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// Task context only. Previous motion stage must finish and release CAN command ownership.
// Remote must be online and must have visited right-DOWN to arm.
// Call start after leaving right-DOWN; request persists until stop, right-DOWN or offline.
void gimbal_tracking_request_start(void);
// Latches stop even if switches are UP. Return right-DOWN before restarting.
void gimbal_tracking_request_stop(void);
#ifdef __cplusplus
}
#endif
