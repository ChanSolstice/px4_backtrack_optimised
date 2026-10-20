#pragma once

#include <cstdint>
#include <vector>

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 配置参数
// ============================================================================

/// LinkMonitor配置参数
struct LinkMonitorConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。

  // 通道18参数
  bool enable_ch18{};              ///< 是否启用通道18失控检测
  int ch18_failsafe_threshold{};   ///< 通道18失控阈值（< 此值视为失控帧）
  double ch18_confirm_s{};         ///< 漏桶确认阈值 (s)，同时是漏桶上限
  double link_loss_leak_rate{};    ///< 通道18正常时漏桶每秒泄漏量

  // RC消息超时参数
  bool enable_rc_loss{};           ///< 是否启用RC消息超时检测
  double rc_loss_timeout_s{};      ///< RC消息超时时间 (s)
  double rc_loss_debounce_s{};     ///< RC超时防抖时间 (s)

  // 恢复与退化参数
  double link_recover_s{};         ///< 链路恢复所需的连续正常时间 (s)
  double link_degraded_window_s{}; ///< 退化判定窗口 (s)
};

// ============================================================================
// 链路状态
// ============================================================================

/// 链路丢失的来源
enum class LinkLossSource {
  None,       ///< 未丢失
  Ch18,       ///< 通道18漏桶确认
  RcTimeout   ///< RC消息超时
};

/// 链路状态快照（电平语义，每次更新后重新计算）
struct LinkStatus {
  bool lost{false};              ///< 链路丢失（锁存，满足恢复条件才清除）
  LinkLossSource source{LinkLossSource::None};  ///< 本次丢失首先确认的来源
  bool degraded{false};          ///< 链路退化：窗口内出现过失控帧或RC超时，或链路丢失
  bool ch18_low_now{false};      ///< 最近一帧通道18低于阈值（该帧通道值不可信）
  bool rc_timeout{false};        ///< 距最近一帧已超过 rc_loss_timeout_s（未防抖）
  double loss_bucket_s{0.0};     ///< 漏桶值 (s)
  bool ch18_unavailable{false};  ///< 启用了通道18，但没有收到RC消息或通道数不足18
  bool ch18_unexpected{false};   ///< 最近一帧通道18既不是正常值也不是失控值
  int ch18_value{-1};            ///< 最近一帧通道18原始值，不可用时为 -1
};

// ============================================================================
// LinkMonitor类
// ============================================================================

/// 链路监测（通道18漏桶、RC消息超时、恢复与退化）
///
/// 只做检测，不做任何状态转移。通道18按帧间保持上一帧的值积分，
/// 结果与 tick 和 RC 帧的先后顺序无关。RC超时只在 update 中判定，
/// 且本模块自身卡顿的那一拍不判定：卡顿期间到达的RC消息还在回调队列里。
class LinkMonitor {
public:
  /// 改动后固件约定的通道18取值：正常帧 1400，失控帧 800
  static constexpr int kCh18NormalValue = 1400;
  static constexpr int kCh18FailsafeValue = 800;

  /// 构造函数
  /// @param config 配置参数
  explicit LinkMonitor(const LinkMonitorConfig& config);

  /// 处理一帧RC消息
  /// @param t 接收时间 (s)
  /// @param channels RC通道数组（从mavros_msgs/RCIn）
  void onRcFrame(double t, const std::vector<uint16_t>& channels);

  /// 推进到当前时间（由20Hz tick调用，用于漏桶积分、RC超时和恢复判定）
  /// @param t 当前时间 (s)
  void update(double t);

  /// 获取当前链路状态
  const LinkStatus& status() const { return status_; }

private:
  /// 按上一帧的通道18状态，把漏桶积分到 t
  void integrate(double t);

  /// 计算恢复和退化
  void evaluate(double t);

  /// 记录一次链路异常（失控帧或RC超时）
  void markBad(double t);

  /// 置位链路丢失（已丢失时保持原来源）
  void setLost(LinkLossSource source);

  /// 配置参数
  LinkMonitorConfig config_;

  /// 当前状态
  LinkStatus status_;

  bool has_frame_{false};        ///< 是否收到过RC消息
  double last_frame_t_{0.0};     ///< 最近一帧的接收时间
  double last_t_{0.0};           ///< 漏桶积分到的时间
  double last_update_t_{0.0};    ///< 上一次 update 的时间（判定自身卡顿）
  bool has_bad_{false};          ///< 是否出现过链路异常
  double last_bad_t_{0.0};       ///< 最近一次链路异常的时间
};

}  // namespace domain
}  // namespace px4_backtrack
