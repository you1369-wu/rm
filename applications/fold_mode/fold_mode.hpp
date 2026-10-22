#ifndef APPLICATIONS_FOLD_MODE_HPP
#define APPLICATIONS_FOLD_MODE_HPP

#include <cstdint>

#include "remote_task.h"

// 每个控制周期调用；包含下档标定、故障确认和折叠控制，不发送 CAN。
// active 只控制折叠运动是否启用，不跳过下档标定及故障处理。
void update_fold_mode(
  const RemoteTaskData & remote_data, bool remote_ready, bool motors_ready,
  uint16_t distance_cm, uint32_t now_ms, bool active = true);

// 仅查询开机标定和折叠故障，公共遥控器及反馈检查由 control_task 负责。
bool fold_mode_ready();

// 公共失能入口调用；清理折叠 PID 和补偿输出，不清除本次上电标定。
void clear_fold_controllers();

#endif  // APPLICATIONS_FOLD_MODE_HPP
