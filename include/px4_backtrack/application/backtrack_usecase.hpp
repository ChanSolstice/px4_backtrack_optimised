#pragma once

#include "px4_backtrack/application/mode_commander.hpp"
#include "px4_backtrack/application/planning_worker.hpp"
#include "px4_backtrack/domain/airborne_monitor.hpp"
#include "px4_backtrack/domain/battery_monitor.hpp"
#include "px4_backtrack/domain/link_monitor.hpp"
#include "px4_backtrack/domain/pilot_input.hpp"
#include "px4_backtrack/domain/recording_buffer.hpp"
#include "px4_backtrack/domain/rejoin_policy.hpp"
#include "px4_backtrack/domain/return_state_machine.hpp"
#include "px4_backtrack/domain/trajectory_session.hpp"
#include "px4_backtrack/domain/waypoint_fallback_session.hpp"

#include <trajectory_core/trajectory_core.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace px4_backtrack {

// Forward declarations for debug helpers
namespace infrastructure { class DebugDumper; }
namespace adapters { class DebugPublisher; }

namespace application {

// ============================================================================
// 配置参数
// ============================================================================

/// BacktrackUseCase配置参数
struct UseCaseConfig {
  // 运行时参数由 ConfigLoader 从 backtrack_params.yaml 注入。

  // 录制
  double record_max_hz{};   ///< 录制最大频率 (Hz)
  int record_min_points{};  ///< 启动任务所需最少录制点数

  // 条件层（重构方案第 5 节）
  domain::LinkMonitorConfig link_config;
  domain::PilotInputConfig pilot_config;
  domain::BatteryMonitorConfig battery_config;
  domain::AirborneMonitorConfig airborne_config;
  double odom_stale_s{};       ///< 里程计延迟 (s)
  double fcu_state_stale_s{};  ///< 飞控状态中断 (s)

  // 状态机与模式请求
  domain::ReturnStateMachineConfig state_machine_config;
  ModeCommanderConfig mode_commander_config;
  double fallback_gap_s{};     ///< 自身断流阈值 (s)（D22）
  double fallback_window_s{};  ///< 断流回看窗口 (s)

  // 执行层
  domain::RejoinConfig rejoin_config;
  trajectory_core::TrajectoryConfig trajectory_config;
  double planning_warn_s{};     ///< 规划告警 (s)：超过后告警 PLANNING_SLOW
  double planning_timeout_s{};  ///< 规划超时 (s)：超过后丢弃结果，改为航点降级
  double cursor_window_m{};     ///< 游标搜索窗口 (m)（第 7.4 节）

  // Debug/可视化（由节点决定是否创建导出器）
  bool debug_enable_csv_dump{};         ///< 启用CSV文件导出
  std::string debug_csv_output_dir;     ///< CSV导出目录
  double debug_trajectory_sample_dt{};  ///< 轨迹采样步长 (s)
  bool debug_enable_rviz_publish{};     ///< 启用ROS可视化发布
};

// ============================================================================
// Tick输出数据
// ============================================================================

/// UseCase tick输出数据
struct TickOutput {
  std::string state;                              ///< 上报的状态字符串（第 6.1 节）
  domain::ControlState control{domain::ControlState::None};  ///< 区域 B
  std::optional<domain::SetpointTarget> setpoint; ///< 本拍发布的 setpoint；为空不发布
  std::optional<std::string> mode_request;        ///< 本拍要发出的模式（异步调用）
  bool link_lost{false};                          ///< 链路丢失（/status_rc_loss）
  bool low_battery{false};                        ///< 低电量（/px4_backtrack/low_battery）
  domain::AlertSet alerts;                        ///< 告警
  int flight_id{0};                               ///< 飞行周期编号
  int mission_id{0};                              ///< 任务编号，没有任务时为 0
  std::vector<domain::StepEvent> events;          ///< 本拍事件（状态机、规划、断流、模式请求）
};

// ============================================================================
// BacktrackUseCase类
// ============================================================================

/// 返航用例（应用层）：组装条件、调用状态机、执行命令（重构方案第 4.1 节）
///
/// 不访问 ROS：各输入的接收时间由调用方传入，服务调用由调用方异步执行。
class BacktrackUseCase {
public:
  /// 构造函数
  /// @param config 配置参数
  /// @param planning_worker 规划器；为空时创建带工作线程的规划器（单测传入不启动线程的，R-76）
  explicit BacktrackUseCase(const UseCaseConfig& config,
                            std::unique_ptr<PlanningWorker> planning_worker = nullptr);

  /// 每个控制周期调用一次
  /// @param now 当前时间 (s)
  TickOutput tick(double now);

  /// 里程计；含非有限值时视为没有收到（R-73）
  /// @param t 接收时间 (s)
  /// @param position 位置 (ENU, m)
  /// @param velocity 线速度 (ENU, m/s)
  /// @param yaw Yaw角 (rad)
  /// @param stamp 消息时间戳 (s)，用于录制
  void onOdom(double t, const domain::Vec3& position, const domain::Vec3& velocity,
              double yaw, double stamp);

  /// RC输入
  void onRc(double t, const std::vector<uint16_t>& channels);

  /// 飞控状态（/mavros/state）；只用 connected 为真的消息（第 5.4 节）
  void onFcuState(double t, bool connected, bool armed, const std::string& mode);

  /// 飞控落地状态（/mavros/extended_state）
  void onExtendedState(double t, domain::LandedState landed_state);

  /// 电池电量百分比 (0.0~1.0)
  void onBattery(double t, double percentage);

  /// 异步模式请求的结果
  /// @param sent 服务调用成功且 mode_sent 为真
  void onModeRequestResult(double t, const std::string& mode, bool sent);

  /// 调试日志是否已失败（R-72）
  void setDebugLogFailed(bool failed) { debug_log_failed_ = failed; }

  /// 设置 CSV debug dumper（可选）
  void setDebugDumper(std::shared_ptr<infrastructure::DebugDumper> dumper);

  /// 设置 ROS debug publisher（可选）
  void setDebugPublisher(std::shared_ptr<adapters::DebugPublisher> publisher);

private:
  /// 用最近一次的输入和执行结果组装条件（R-71）
  domain::Conditions assembleConditions(double now) const;

  /// 执行状态机本拍的命令
  void executeCommands(const domain::StepOutput& step, double now,
                       std::vector<domain::StepEvent>& events);

  /// 基于当前录制缓存提交后台规划（第 7.5 节）
  void submitPlanning(double now, std::vector<domain::StepEvent>& events);

  /// 取规划结果；规划超时时丢弃结果，在控制线程直接构建航点降级
  void receivePlanning(double now, std::vector<domain::StepEvent>& events);

  /// 采用规划结果
  void applyPlanningResult(PlanningResult result,
                           std::vector<domain::StepEvent>& events);

  /// 丢弃规划结果与跟随状态
  void resetPlan();

  /// 推进跟随进度，更新偏离与回归判定
  void updateFollow(const domain::StepOutput& step, double now);

  /// 游标跟着跟随进度向起点移动：轨迹用路径投影点，航点降级用飞机位置（第 7.4 节）
  void updateCursor();

  /// 找点接上：游标之前离飞机不超过安全距离的录制点里最早录制的一个（D33）
  std::optional<size_t> findAnchor() const;

  /// 录制截断到 last（含），记录事件
  void cutTrail(size_t last, std::vector<domain::StepEvent>& events);

  /// 按 setpoint 策略计算目标；拿不到目标时保持当前位置（P3）
  std::optional<domain::SetpointTarget> computeSetpoint(
      const domain::StepOutput& step);

  /// 记录本拍是否发布，计算自身断流（R-69）
  void trackPublishing(double now, bool published,
                       std::vector<domain::StepEvent>& events);
  bool streamGapRecent(double now) const;

  /// 配置参数
  UseCaseConfig config_;

  // 条件层
  domain::LinkMonitor link_monitor_;
  domain::PilotInput pilot_input_;
  domain::BatteryMonitor battery_monitor_;
  domain::AirborneMonitor airborne_monitor_;

  // 决策
  domain::ReturnStateMachine state_machine_;
  ModeCommander mode_commander_;

  // 执行层
  domain::RecordingBuffer recording_buffer_;
  std::unique_ptr<PlanningWorker> planning_worker_;
  std::unique_ptr<domain::TrajectorySession> trajectory_session_;  ///< 规划就绪后有效
  domain::WaypointFallbackSession waypoint_fallback_session_;
  domain::RejoinPolicy rejoin_policy_;
  domain::PlanStatus plan_status_{domain::PlanStatus::None};
  double plan_submit_t_{0.0};    ///< 本次规划的提交时间
  size_t plan_begin_{0};         ///< 规划用的连续段在录制中的起点，大于 0 表示有断点（D30）
  bool plan_timed_out_{false};   ///< 本任务的航点降级来自规划超时
  size_t cursor_{0};             ///< 游标：返航已经走到的录制点（第 7.4 节）

  // 最近一次输入
  bool has_fcu_state_{false};
  double fcu_state_t_{0.0};
  bool armed_{false};
  std::string mode_;
  bool has_odom_{false};
  double odom_t_{0.0};
  domain::Vec3 position_ = domain::Vec3::Zero();
  domain::Vec3 velocity_ = domain::Vec3::Zero();
  double yaw_{0.0};
  bool debug_log_failed_{false};

  /// 上一拍是否录制（决定之后的里程计是否写入录制缓存）
  bool record_{false};

  // 偏离与回归的持续判定
  domain::FollowMode last_follow_{domain::FollowMode::OnPath};
  bool deviation_confirmed_{false};
  bool rejoin_confirmed_{false};
  double deviation_since_{-1.0};
  double rejoin_since_{-1.0};

  // 上一次发布的路径目标速度，用于 Rejoin 速度连续性
  domain::Vec3 last_path_velocity_ = domain::Vec3::Zero();
  bool has_last_path_velocity_{false};

  // 自身断流（R-69）：只在连续发布期间计算
  bool publishing_{false};
  double last_publish_t_{0.0};
  std::optional<double> last_gap_end_t_;

  // Debug helpers（可选，nullptr 表示禁用），随规划请求交给规划线程
  std::shared_ptr<infrastructure::DebugDumper> debug_dumper_;
  std::shared_ptr<adapters::DebugPublisher> debug_publisher_;
};

}  // namespace application
}  // namespace px4_backtrack
