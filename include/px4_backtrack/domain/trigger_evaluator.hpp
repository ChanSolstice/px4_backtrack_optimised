#pragma once

#include <vector>
#include <cstdint>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// TriggerEvaluator配置参数
struct TriggerConfig {
  // 通道7参数
  bool enable_ch7;               ///< 是否启用通道7触发
  int ch7_pressed_threshold;     ///< 通道7按下阈值（< 此值视为按下）
  double ch7_hold_s;             ///< 长按持续时间 (s)

  // 通道18参数
  bool enable_ch18;              ///< 是否启用通道18触发
  int ch18_failsafe_threshold;   ///< 通道18故障阈值（< 此值视为失控）
  double ch18_confirm_s;         ///< 故障确认时间 (s)

  // RC丢失参数
  bool enable_rc_loss;           ///< 是否启用RC丢失触发
  double rc_loss_timeout_s;      ///< RC消息超时时间 (s)
  double rc_loss_debounce_s;     ///< RC丢失防抖时间 (s)

  /// 默认构造函数
  TriggerConfig()
    : enable_ch7(true),
      ch7_pressed_threshold(1500),
      ch7_hold_s(2.0),
      enable_ch18(true),
      ch18_failsafe_threshold(900),
      ch18_confirm_s(1.0),
      enable_rc_loss(true),
      rc_loss_timeout_s(0.15),
      rc_loss_debounce_s(0.1)
  {}
};

// ============================================================================
// TriggerEvaluator类
// ============================================================================

/// 触发判定器（通道7/18/RC丢失）
class TriggerEvaluator {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit TriggerEvaluator(const TriggerConfig& config);

  /// 更新RC通道数据
  /// @param channels RC通道数组（从mavros_msgs/RCIn）
  void updateRC(const std::vector<uint16_t>& channels);

  /// 更新最近一次收到RC消息的时间
  /// @param t_now 当前时间 (s)
  void updateLastRcMsgTime(double t_now);

  /// 更新当前时间（由20Hz tick调用，用于RC丢失判定）
  /// @param t_now 当前时间 (s)
  void updateNow(double t_now);

  /// 检测通道7长按（启动/恢复返航）
  /// @return 是否检测到长按
  bool checkCh7LongPress();

  /// 检测通道7按下事件（暂停返航）
  /// @return 是否检测到按下事件
  bool checkCh7PressEvent();

  /// 检测通道18故障确认
  /// @return 是否确认故障
  bool checkCh18Failsafe();

  /// 检测RC消息丢失
  /// @return 是否检测到RC丢失
  bool checkRCLoss();

private:
  /// 配置参数
  TriggerConfig config_;

  // 通道7状态
  enum class Ch7State {
    Released,   ///< 未按下
    Pressed,    ///< 已按下
    LongPress   ///< 长按
  };

  Ch7State ch7_state_;            ///< 通道7当前状态
  double ch7_press_start_time_;   ///< 通道7按下开始时间
  bool ch7_press_event_fired_;    ///< 按下事件是否已触发（防止重复触发）

  // 通道18状态
  bool ch18_failsafe_active_;     ///< 通道18是否处于失控状态
  double ch18_failsafe_start_time_; ///< 失控开始时间
  bool ch18_confirmed_;           ///< 是否已确认

  // RC丢失状态
  double last_rc_msg_time_;       ///< 最近一次收到RC消息的时间
  double now_;                    ///< 当前时间
  bool rc_loss_debouncing_;       ///< 是否处于防抖期
  double rc_loss_debounce_start_time_; ///< 防抖开始时间
  bool rc_loss_triggered_;        ///< 是否已触发RC丢失
  bool has_received_rc_;          ///< 是否收到过RC消息（启动保护）

  // 当前RC通道值
  std::vector<uint16_t> current_channels_;

  /// 获取通道7的值
  /// @return 通道7的值，如果不存在返回中立值1500
  uint16_t getCh7Value() const;

  /// 获取通道18的值
  /// @return 通道18的值，如果不存在返回正常值1500
  uint16_t getCh18Value() const;
};

}  // namespace domain
}  // namespace px4_backtrack
