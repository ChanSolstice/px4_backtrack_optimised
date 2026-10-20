#include "px4_backtrack/domain/link_monitor.hpp"

#include <algorithm>

namespace px4_backtrack {
namespace domain {

namespace {

/// 通道18在通道数组中的索引（从0开始）
constexpr std::size_t kCh18Index = 17;

/// 时间比较容差，避免浮点误差让边界判定晚一帧
constexpr double kTimeEpsS = 1e-9;

}  // namespace

LinkMonitor::LinkMonitor(const LinkMonitorConfig& config)
  : config_(config)
{
  // 还没有收到RC消息时，通道18同样视为不可用
  status_.ch18_unavailable = config_.enable_ch18;
}

void LinkMonitor::onRcFrame(double t, const std::vector<uint16_t>& channels) {
  // 先按上一帧的状态积分到本帧时刻，再换成本帧的状态
  integrate(t);
  has_frame_ = true;
  last_frame_t_ = t;
  status_.rc_timeout = false;
  if (in_rc_timeout_) {
    // RC超时到本帧为止，连续正常从本帧算起
    in_rc_timeout_ = false;
    markBad(t);
  }

  if (config_.enable_ch18) {
    if (channels.size() > kCh18Index) {
      const int value = channels[kCh18Index];
      status_.ch18_unavailable = false;
      status_.ch18_value = value;
      status_.ch18_low_now = value < config_.ch18_failsafe_threshold;
      status_.ch18_unexpected =
          value != kCh18NormalValue && value != kCh18FailsafeValue;
    } else {
      // 通道数不足：通道18不参与判定，既不算失控帧，也不算退化
      status_.ch18_unavailable = true;
      status_.ch18_value = -1;
      status_.ch18_low_now = false;
      status_.ch18_unexpected = false;
    }
    if (status_.ch18_low_now) {
      markBad(t);
    }
  }

  evaluate(t);
}

void LinkMonitor::update(double t) {
  integrate(t);

  // 两次 update 间隔超过RC超时，说明本模块自身卡顿过：这期间到达的RC消息
  // 可能还在回调队列里，没到达不代表链路中断，这一拍不判RC超时。
  const bool self_stalled =
      !has_update_ || t - last_update_t_ + kTimeEpsS >= config_.rc_loss_timeout_s;
  has_update_ = true;
  last_update_t_ = t;

  const double since_frame = t - last_frame_t_;
  status_.rc_timeout =
      has_frame_ && since_frame + kTimeEpsS >= config_.rc_loss_timeout_s;
  if (config_.enable_rc_loss && status_.rc_timeout && !self_stalled) {
    in_rc_timeout_ = true;
    markBad(t);
    if (since_frame + kTimeEpsS >=
        config_.rc_loss_timeout_s + config_.rc_loss_debounce_s) {
      setLost(LinkLossSource::RcTimeout);
    }
  }

  evaluate(t);
}

void LinkMonitor::integrate(double t) {
  if (!has_time_) {
    has_time_ = true;
    last_t_ = t;
    return;
  }
  const double dt = t - last_t_;
  if (dt <= 0.0) {
    return;
  }
  last_t_ = t;

  if (status_.ch18_low_now) {
    status_.loss_bucket_s =
        std::min(status_.loss_bucket_s + dt, config_.ch18_confirm_s);
    markBad(t);
    if (status_.loss_bucket_s + kTimeEpsS >= config_.ch18_confirm_s) {
      setLost(LinkLossSource::Ch18);
    }
  } else {
    status_.loss_bucket_s = std::max(
        0.0, status_.loss_bucket_s - config_.link_loss_leak_rate * dt);
  }
}

void LinkMonitor::evaluate(double t) {
  // 恢复：通道18连续正常至少 link_recover_s，且RC消息新鲜；漏桶不清零
  const bool frames_fresh =
      has_frame_ && t - last_frame_t_ < config_.rc_loss_timeout_s;
  if (status_.lost && frames_fresh && !status_.ch18_low_now &&
      t - last_bad_t_ + kTimeEpsS >= config_.link_recover_s) {
    status_.lost = false;
    status_.source = LinkLossSource::None;
  }

  status_.degraded = status_.lost ||
      (has_bad_ && t - last_bad_t_ < config_.link_degraded_window_s);
}

void LinkMonitor::markBad(double t) {
  has_bad_ = true;
  last_bad_t_ = std::max(last_bad_t_, t);
}

void LinkMonitor::setLost(LinkLossSource source) {
  if (!status_.lost) {
    status_.lost = true;
    status_.source = source;
  }
}

}  // namespace domain
}  // namespace px4_backtrack
