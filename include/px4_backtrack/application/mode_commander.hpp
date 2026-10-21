#pragma once

#include <optional>
#include <string>
#include <vector>

#include "px4_backtrack/domain/return_state_machine.hpp"

namespace px4_backtrack {
namespace application {

// ============================================================================
// 配置参数
// ============================================================================

/// ModeCommander配置参数
struct ModeCommanderConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。
  double request_interval_s{};  ///< 意图不变时的重发间隔 (s)
  double request_timeout_s{};   ///< 从第一次发出算起，多久未观察到目标模式视为失败 (s)
};

// ============================================================================
// 模式请求状态
// ============================================================================

/// 当前模式请求的结果
enum class ModeRequestResult {
  None,     ///< 没有意图
  Pending,  ///< 已发出，等待在飞控状态中观察到
  Applied,  ///< 已观察到目标模式
  Failed    ///< 超时未生效、mode_sent 为假或服务不可用（继续按间隔重发）
};

/// 模式请求状态（上报）
struct ModeRequestStatus {
  std::string mode;      ///< 当前意图（MAVROS 模式字符串，空为无）
  double sent_at{0.0};   ///< 最近一次发出的时间 (s)
  ModeRequestResult result{ModeRequestResult::None};
};

const char* modeRequestResultName(ModeRequestResult result);

// ============================================================================
// ModeCommander类
// ============================================================================

/// 模式请求（重构方案第 7.7 节，D24）
///
/// 意图改变时立即发送；意图不变时按间隔重发，直到在飞控状态中观察到目标模式。
/// 纯逻辑：不访问 ROS，服务调用由调用方异步执行，结果用 onResult 送回。
class ModeCommander {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit ModeCommander(const ModeCommanderConfig& config);

  /// 每个控制周期调用一次
  /// @param now 当前时间 (s)
  /// @param intent 状态机输出的模式意图（空为无）
  /// @param fcu_mode 最近一次收到的飞控模式
  /// @param fcu_state_fresh 飞控状态是否新鲜；中断时无法确认生效，只按间隔重发
  /// @return 本拍需要发出的模式；不需要发送时为空
  std::optional<std::string> update(double now, const std::string& intent,
                                    const std::string& fcu_mode,
                                    bool fcu_state_fresh);

  /// 异步服务调用的结果
  /// @param now 当前时间 (s)
  /// @param mode 请求的模式
  /// @param sent 服务调用成功且 mode_sent 为真
  void onResult(double now, const std::string& mode, bool sent);

  /// 当前请求状态
  const ModeRequestStatus& status() const { return status_; }

  /// 当前请求是否失败（MODE_REQUEST_FAILED）
  bool failed() const { return status_.result == ModeRequestResult::Failed; }

  /// 取出上次调用以来的事件（意图变化、发出、生效、失败）
  std::vector<domain::StepEvent> takeEvents();

private:
  /// 发出当前意图
  std::string send(double now, const char* reason);

  /// 记录结果变化
  void setResult(ModeRequestResult result, const char* reason);

  /// 配置参数
  ModeCommanderConfig config_;

  /// 当前请求状态
  ModeRequestStatus status_;

  double first_sent_t_{0.0};  ///< 当前这一轮申请第一次发出的时间（超时从此算起，R-70）
  std::vector<domain::StepEvent> events_;
};

}  // namespace application
}  // namespace px4_backtrack
