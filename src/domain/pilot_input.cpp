#include "px4_backtrack/domain/pilot_input.hpp"

namespace px4_backtrack {
namespace domain {

namespace {

/// 通道7在通道数组中的索引（从0开始）
constexpr std::size_t kCh7Index = 6;

/// 时间比较容差，避免浮点误差让边界判定晚一帧
constexpr double kTimeEpsS = 1e-9;

}  // namespace

PilotInput::PilotInput(const PilotInputConfig& config)
  : config_(config)
{
}

void PilotInput::onRcFrame(double t,
                           const std::vector<uint16_t>& channels,
                           const LinkStatus& link) {
  if (!config_.enable_ch7) {
    return;
  }

  // RC超时后到达的帧与之前的帧不相连：之前的按下作废，本帧不能构成按下沿
  const bool continuous = has_frame_ &&
      t - last_frame_t_ + kTimeEpsS < config_.rc_loss_timeout_s;
  has_frame_ = true;
  last_frame_t_ = t;
  if (!continuous) {
    cancelPress();
  }

  // 失控帧：通道值是接收机的失控输出，不可信
  if (link.ch18_low_now) {
    cancelPress();
    return;
  }

  // 通道数不足时按松开处理
  const bool pressed = channels.size() > kCh7Index &&
      channels[kCh7Index] < config_.ch7_pressed_threshold;

  if (status_.pressed) {
    if (pressed) {
      trackPress(t, link);
      return;
    }

    // 松开：按下时间在 [ch7_exit_min_s, ch7_hold_s) 内为短按，松开时产生
    press_link_clean_ = press_link_clean_ && !link.degraded;
    const double duration = t - status_.press_start_t;
    if (!long_fired_ &&
        duration + kTimeEpsS >= config_.ch7_exit_min_s &&
        duration + kTimeEpsS < config_.ch7_hold_s) {
      status_.exit_request = Ch7Command{
          status_.press_id, status_.press_start_t, t, press_link_clean_};
    }
    status_.pressed = false;
    long_fired_ = false;
    status_.wait_release = false;
    return;
  }

  if (!pressed) {
    // 正常帧中的松开：可以接受下一次按下
    status_.wait_release = false;
    return;
  }

  // 新按下：前一帧是相连的正常松开帧（wait_release 已清除），当前帧是正常按下帧
  if (!status_.wait_release) {
    ++status_.press_id;
    status_.pressed = true;
    status_.press_start_t = t;
    long_fired_ = false;
    press_link_clean_ = true;
    trackPress(t, link);
  }
}

void PilotInput::update(double t, const LinkStatus& link) {
  if (!config_.enable_ch7) {
    return;
  }

  // RC超时：之前的按下作废
  if (has_frame_ && t - last_frame_t_ + kTimeEpsS >= config_.rc_loss_timeout_s) {
    cancelPress();
    return;
  }

  if (status_.pressed) {
    trackPress(t, link);
  }
}

void PilotInput::reset() {
  cancelPress();
  status_.return_request.reset();
  status_.exit_request.reset();
}

void PilotInput::cancelPress() {
  status_.pressed = false;
  status_.wait_release = true;
  long_fired_ = false;
}

void PilotInput::trackPress(double t, const LinkStatus& link) {
  press_link_clean_ = press_link_clean_ && !link.degraded;

  if (!long_fired_ &&
      t - status_.press_start_t + kTimeEpsS >= config_.ch7_hold_s) {
    long_fired_ = true;
    status_.return_request = Ch7Command{
        status_.press_id, status_.press_start_t, t, !link.lost};
  }
}

}  // namespace domain
}  // namespace px4_backtrack
