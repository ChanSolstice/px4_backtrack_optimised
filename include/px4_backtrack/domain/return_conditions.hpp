#pragma once

#include <optional>
#include <string>

#include "px4_backtrack/domain/airborne_monitor.hpp"
#include "px4_backtrack/domain/battery_monitor.hpp"
#include "px4_backtrack/domain/link_monitor.hpp"
#include "px4_backtrack/domain/pilot_input.hpp"
#include "px4_backtrack/domain/types.hpp"

namespace px4_backtrack {
namespace domain {

// ============================================================================
// 执行事实
// ============================================================================

/// 规划状态（执行层提供，重构方案第 5.6、7.5 节）
enum class PlanStatus {
  None,            ///< 未提交
  Pending,         ///< 规划中
  Ready,           ///< 轨迹就绪
  ReadyFallback,   ///< 航点降级就绪
  FailedTooClose,  ///< 点过近：原地降落
  FailedConfig,    ///< 配置非法：保持并告警
  FailedFallback,  ///< 航点降级也构建失败：原地降落
  Timeout          ///< 规划超时，正在改为航点降级
};

// ============================================================================
// Conditions快照
// ============================================================================

/// 每个周期的条件快照（重构方案第 5 节），由 UseCase 组装，状态机只读
struct Conditions {
  double now{0.0};  ///< 当前时间 (s)

  // 飞控状态（/mavros/state，只用 connected 为真的消息）
  bool fcu_state_fresh{false};  ///< 距最近一次飞控状态未超过 fcu_state_stale_s
  bool armed{false};            ///< 是否解锁（飞控状态中断时为最后一次的值）
  std::string mode;             ///< 飞控模式（MAVROS 字符串，如 "OFFBOARD"、"AUTO.LAND"）

  // 条件层
  LinkStatus link;              ///< 链路
  PilotInputStatus pilot;       ///< CH7 指令
  BatteryStatus battery;        ///< 电量
  AirborneStatus airborne;      ///< 在空中判定

  // 里程计
  bool odom_stale{true};        ///< 距最近一次里程计超过 odom_stale_s
  bool odom_lost{true};         ///< 距最近一次里程计超过 odom_timeout_s
  Vec3 position = Vec3::Zero(); ///< 最近一次里程计位置 (ENU, m)
  double yaw{0.0};              ///< 最近一次里程计 yaw (rad)

  // 执行事实（第 5.6 节）
  bool has_min_points{false};             ///< 录制点数 ≥ record_min_points
  PlanStatus plan_status{PlanStatus::None};
  std::optional<double> path_start_yaw;   ///< 轨迹就绪后的路径起始切向；航点降级没有，不要求对准
  std::optional<double> path_target_yaw;  ///< 跟随阶段当前路径目标的切向（重新接入时对准）
  bool deviation_confirmed{false};        ///< 偏离已持续确认，或局部投影失败
  bool rejoin_confirmed{false};           ///< 回归已持续确认
  bool finished{false};                   ///< 轨迹或航点降级会话完成
  bool projection_ok{false};              ///< 局部投影有效（航点降级按 true）
  double cross_track_m{0.0};              ///< 横向偏差 (m)（航点降级按 0）
  bool stream_gap_recent{false};          ///< 最近 fallback_window_s 内本模块发布间隔 ≥ fallback_gap_s
  bool mode_request_failed{false};        ///< 最近一次模式请求超时未生效、被拒或服务不可用
  bool debug_log_failed{false};          ///< 调试日志写入失败，已停止写日志（R-72）
};

}  // namespace domain
}  // namespace px4_backtrack
