#include "px4_backtrack/application/mode_commander.hpp"

namespace px4_backtrack {
namespace application {

namespace {

/// 时间比较容差，避免浮点误差让边界判定晚一拍
constexpr double kTimeEpsS = 1e-9;

domain::StepEvent makeEvent(const char* name, std::string from, std::string to,
                            const char* reason) {
  domain::StepEvent event;
  event.type = domain::EventType::ModeRequest;
  event.name = name;
  event.from = std::move(from);
  event.to = std::move(to);
  event.reason = reason;
  return event;
}

}  // namespace

const char* modeRequestResultName(ModeRequestResult result) {
  switch (result) {
    case ModeRequestResult::None: return "none";
    case ModeRequestResult::Pending: return "pending";
    case ModeRequestResult::Applied: return "applied";
    case ModeRequestResult::Failed: return "failed";
  }
  return "unknown";
}

ModeCommander::ModeCommander(const ModeCommanderConfig& config)
  : config_(config)
{
}

std::optional<std::string> ModeCommander::update(double now,
                                                 const std::string& intent,
                                                 const std::string& fcu_mode,
                                                 bool fcu_state_fresh) {
  const bool observed = fcu_state_fresh && fcu_mode == intent;

  if (intent != status_.mode) {
    // 意图改变：重新计时；已经是目标模式时不发送
    events_.push_back(makeEvent("intent", status_.mode, intent, ""));
    status_ = ModeRequestStatus{};
    status_.mode = intent;
    if (intent.empty()) {
      return std::nullopt;
    }
    if (observed) {
      setResult(ModeRequestResult::Applied, "ALREADY_IN_MODE");
      return std::nullopt;
    }
    first_sent_t_ = now;
    return send(now, "INTENT_CHANGED");
  }

  if (intent.empty()) {
    return std::nullopt;
  }

  if (observed) {
    setResult(ModeRequestResult::Applied, "OBSERVED");
    return std::nullopt;
  }

  if (status_.result == ModeRequestResult::Applied) {
    if (!fcu_state_fresh) {
      return std::nullopt;
    }
    // 生效后又被切走：重新开始一轮申请
    first_sent_t_ = now;
    status_.result = ModeRequestResult::Pending;
    return send(now, "MODE_LOST");
  }

  if (status_.result == ModeRequestResult::Pending &&
      now - first_sent_t_ + kTimeEpsS >= config_.request_timeout_s) {
    setResult(ModeRequestResult::Failed, "TIMEOUT");
  }
  if (now - status_.sent_at + kTimeEpsS >= config_.request_interval_s) {
    return send(now, "RESEND");
  }
  return std::nullopt;
}

void ModeCommander::onResult(double now, const std::string& mode, bool sent) {
  (void)now;
  // 旧意图的结果、已生效后的结果不再改变状态
  if (sent || mode != status_.mode ||
      status_.result == ModeRequestResult::Applied) {
    return;
  }
  setResult(ModeRequestResult::Failed, "NOT_SENT");
}

std::vector<domain::StepEvent> ModeCommander::takeEvents() {
  std::vector<domain::StepEvent> events;
  events.swap(events_);
  return events;
}

std::string ModeCommander::send(double now, const char* reason) {
  status_.sent_at = now;
  if (status_.result == ModeRequestResult::None) {
    status_.result = ModeRequestResult::Pending;
  }
  events_.push_back(makeEvent("sent", "", status_.mode, reason));
  return status_.mode;
}

void ModeCommander::setResult(ModeRequestResult result, const char* reason) {
  if (result == status_.result) {
    return;
  }
  events_.push_back(makeEvent(modeRequestResultName(result),
                              modeRequestResultName(status_.result),
                              status_.mode, reason));
  status_.result = result;
}

}  // namespace application
}  // namespace px4_backtrack
