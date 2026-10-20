#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "px4_backtrack/domain/link_monitor.hpp"

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// PilotInput配置参数
struct PilotInputConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  bool enable_ch7{};            ///< 是否启用通道7
  int ch7_pressed_threshold{};  ///< 通道7按下阈值（< 此值视为按下）
  double ch7_hold_s{};          ///< 长按时间 (s)，达到时产生 return_request
  double ch7_exit_min_s{};      ///< 退出短按的最短按下时间 (s)
  double rc_loss_timeout_s{};   ///< RC消息超时时间 (s)：帧间隔达到此值时作废当前按下
};

// ============================================================================
// 通道7指令
// ============================================================================

/// 一条通道7指令。同一次按下最多产生一条指令，以 press_id 标识，由状态机消费。
struct Ch7Command {
  int press_id{0};              ///< 产生指令的那次按下的编号
  double press_start_t{0.0};    ///< 按下开始时间 (s)
  double t{0.0};                ///< 指令产生时间 (s)：长按为达到 ch7_hold_s 的时刻，短按为松开的时刻
  bool link_ok{false};          ///< 长按：此时链路未丢失；短按：按下全程链路正常且未退化
};

/// 飞手输入状态快照
struct PilotInputStatus {
  bool pressed{false};          ///< 有一次有效按下正在计时
  int press_id{0};              ///< 最近一次有效按下的编号（从1开始递增，复位也不归零）
  double press_start_t{0.0};    ///< 最近一次有效按下的开始时间 (s)
  bool wait_release{true};      ///< 须先在正常帧中看到松开，才接受下一次按下
  std::optional<Ch7Command> return_request;  ///< 最近一次长按，保留到下一次长按或复位
  std::optional<Ch7Command> exit_request;    ///< 最近一次短按，保留到下一次短按或复位
};

// ============================================================================
// PilotInput类
// ============================================================================

/// 飞手输入（通道7长按/短按，逐帧链路门控）
///
/// 失控帧（同帧通道18低）或RC超时期间，通道值是接收机的失控输出或已经过时：
/// 不计时、不产生指令，当前按下作废；之后须先在正常帧中看到松开，才接受下一次按下。
/// 只给出指令和链路标志，是否响应由状态机决定。
class PilotInput {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit PilotInput(const PilotInputConfig& config);

  /// 处理一帧RC消息（须在 LinkMonitor 处理同一帧之后调用）
  /// @param t 接收时间 (s)
  /// @param channels RC通道数组（从mavros_msgs/RCIn）
  /// @param link 处理完本帧后的链路状态
  void onRcFrame(double t,
                 const std::vector<uint16_t>& channels,
                 const LinkStatus& link);

  /// 推进到当前时间（由20Hz tick调用，在 LinkMonitor::update 之后）
  /// @param t 当前时间 (s)
  /// @param link 当前链路状态
  void update(double t, const LinkStatus& link);

  /// 新的飞行周期：作废进行中的按下（须先松开），清除未消费的指令；press_id 不归零
  void reset();

  /// 获取当前状态
  const PilotInputStatus& status() const { return status_; }

private:
  /// 作废进行中的按下，并要求先松开
  void cancelPress();

  /// 按下期间：累计链路状态，检查是否达到长按
  void trackPress(double t, const LinkStatus& link);

  /// 配置参数
  PilotInputConfig config_;

  /// 当前状态
  PilotInputStatus status_;

  double last_frame_t_{0.0};      ///< 最近一帧的接收时间
  bool long_fired_{false};        ///< 本次按下已产生长按指令
  bool press_link_clean_{false};  ///< 本次按下至今链路正常且未退化
};

}  // namespace domain
}  // namespace px4_backtrack
