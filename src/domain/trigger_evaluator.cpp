#include "px4_backtrack/domain/trigger_evaluator.hpp"

namespace px4_backtrack {
namespace domain {

TriggerEvaluator::TriggerEvaluator(const TriggerConfig& config)
  : config_(config),
    ch7_state_(Ch7State::Released),
    ch7_press_start_time_(0.0),
    ch7_press_event_fired_(false),
    ch18_failsafe_active_(false),
    ch18_failsafe_start_time_(0.0),
    ch18_confirmed_(false),
    last_rc_msg_time_(0.0),
    now_(0.0),
    rc_loss_debouncing_(false),
    rc_loss_debounce_start_time_(0.0),
    rc_loss_triggered_(false),
    has_received_rc_(false)
{
}

void TriggerEvaluator::updateRC(const std::vector<uint16_t>& channels) {
  current_channels_ = channels;
}

void TriggerEvaluator::updateLastRcMsgTime(double t_now) {
  last_rc_msg_time_ = t_now;
  has_received_rc_ = true;  // 标记已收到RC消息

  // 收到RC消息，重置丢失相关状态
  if (rc_loss_debouncing_ || rc_loss_triggered_) {
    rc_loss_debouncing_ = false;
    rc_loss_triggered_ = false;
  }
}

void TriggerEvaluator::updateNow(double t_now) {
  now_ = t_now;
}

uint16_t TriggerEvaluator::getCh7Value() const {
  // 通道7对应channels[6]（索引从0开始）
  if (current_channels_.size() > 6) {
    return current_channels_[6];
  }
  return 1500;  // 默认中立值
}

uint16_t TriggerEvaluator::getCh18Value() const {
  // 通道18对应channels[17]（索引从0开始）
  if (current_channels_.size() > 17) {
    return current_channels_[17];
  }
  return 1500;  // 默认正常值
}

bool TriggerEvaluator::checkCh7LongPress() {
  if (!config_.enable_ch7) {
    return false;
  }

  uint16_t ch7 = getCh7Value();
  bool is_pressed = (ch7 < config_.ch7_pressed_threshold);

  // 状态机转换
  switch (ch7_state_) {
    case Ch7State::Released:
      if (is_pressed) {
        // 从未按下转为按下
        ch7_state_ = Ch7State::Pressed;
        ch7_press_start_time_ = now_;
        ch7_press_event_fired_ = false;  // 重置事件标志
      }
      break;

    case Ch7State::Pressed:
      if (!is_pressed) {
        // 释放按钮，回到未按下状态
        ch7_state_ = Ch7State::Released;
      } else {
        // 持续按下，检查是否达到长按时间
        double hold_duration = now_ - ch7_press_start_time_;
        if (hold_duration >= config_.ch7_hold_s) {
          ch7_state_ = Ch7State::LongPress;
          return true;  // 触发长按事件
        }
      }
      break;

    case Ch7State::LongPress:
      if (!is_pressed) {
        // 释放按钮，回到未按下状态
        ch7_state_ = Ch7State::Released;
      }
      // 长按状态不重复触发
      break;
  }

  return false;
}

bool TriggerEvaluator::checkCh7PressEvent() {
  if (!config_.enable_ch7) {
    return false;
  }

  uint16_t ch7 = getCh7Value();
  bool is_pressed = (ch7 < config_.ch7_pressed_threshold);

  // 检测"未按下 -> 按下"的边缘
  if (ch7_state_ == Ch7State::Pressed && !ch7_press_event_fired_ && is_pressed) {
    ch7_press_event_fired_ = true;
    return true;
  }

  return false;
}

bool TriggerEvaluator::checkCh18Failsafe() {
  if (!config_.enable_ch18) {
    return false;
  }

  uint16_t ch18 = getCh18Value();
  bool is_failsafe = (ch18 < config_.ch18_failsafe_threshold);

  if (is_failsafe) {
    if (!ch18_failsafe_active_) {
      // 开始失控
      ch18_failsafe_active_ = true;
      ch18_failsafe_start_time_ = now_;
      ch18_confirmed_ = false;
    } else {
      // 持续失控，检查是否达到确认时间
      if (!ch18_confirmed_) {
        double failsafe_duration = now_ - ch18_failsafe_start_time_;
        if (failsafe_duration >= config_.ch18_confirm_s) {
          ch18_confirmed_ = true;
          return true;  // 触发故障确认
        }
      }
    }
  } else {
    // 恢复正常，重置状态
    if (ch18_failsafe_active_) {
      ch18_failsafe_active_ = false;
      ch18_confirmed_ = false;
    }
  }

  return false;
}

bool TriggerEvaluator::checkRCLoss() {
  if (!config_.enable_rc_loss) {
    return false;
  }

  // 启动保护：只有收到过RC消息后才检测丢失
  if (!has_received_rc_) {
    return false;
  }

  // 计算距离上次收到RC消息的时间
  double time_since_last_msg = now_ - last_rc_msg_time_;

  // 超时检测
  if (time_since_last_msg >= config_.rc_loss_timeout_s) {
    if (!rc_loss_debouncing_ && !rc_loss_triggered_) {
      // 开始防抖
      rc_loss_debouncing_ = true;
      rc_loss_debounce_start_time_ = now_;
    }

    if (rc_loss_debouncing_ && !rc_loss_triggered_) {
      // 检查防抖时间
      double debounce_duration = now_ - rc_loss_debounce_start_time_;
      if (debounce_duration >= config_.rc_loss_debounce_s) {
        rc_loss_triggered_ = true;
        return true;  // 触发RC丢失
      }
    }
  } else {
    // 未超时，重置状态
    if (rc_loss_debouncing_) {
      rc_loss_debouncing_ = false;
    }
  }

  return false;
}

}  // namespace domain
}  // namespace px4_backtrack
