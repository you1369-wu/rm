#pragma once

#include <cstdint>

namespace mission_config
{
// 自动流程实际开始后，先运行原折叠模式 15秒，再请求循迹模式。
constexpr uint32_t kFoldDurationMs = 15000U;
// 自动运行时控制任务长时间未被调度，锁存失能，不能跨过停顿直接切换阶段。
constexpr uint32_t kMaxControlGapMs = 5U;
}  // namespace mission_config
