#ifndef APPLICATIONS_FOLD_MODE_HPP
#define APPLICATIONS_FOLD_MODE_HPP

#include <cstdint>

#include "remote_task.h"

// 每个控制周期调用；包含下档标定、故障确认和折叠控制，不发送 CAN。
void update_fold_mode(
  const RemoteTaskData & remote_data, bool remote_ready, bool motors_ready,
  uint16_t distance_cm, uint32_t now_ms);

// 公共失能入口调用；清理折叠 PID 和补偿输出，不清除本次上电标定。
void clear_fold_controllers();

#endif  // APPLICATIONS_FOLD_MODE_HPP
