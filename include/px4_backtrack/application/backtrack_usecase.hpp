#pragma once

#include "px4_backtrack/domain/state_machine.hpp"
#include "px4_backtrack/domain/recording_buffer.hpp"
#include "px4_backtrack/domain/trigger_evaluator.hpp"
#include "px4_backtrack/domain/trajectory_session.hpp"
#include "px4_backtrack/domain/waypoint_fallback_session.hpp"
#include "px4_backtrack/domain/rejoin_policy.hpp"

#include <trajectory_core/trajectory_core.hpp>
#include <memory>
#include <string>

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
  // 录制参数
  double record_delay_s;          ///< ARM后延时录制时间 (s)
  double record_max_hz;           ///< 录制最大频率 (Hz)
  int record_min_points;          ///< 触发返航所需最少录制点数

  // 状态机参数
  domain::StateMachineConfig state_machine_config;

  // 触发参数
  domain::TriggerConfig trigger_config;

  // 回归参数
  domain::RejoinConfig rejoin_config;

  // trajectory_core配置
  trajectory_core::TrajectoryConfig trajectory_config;

  // Debug/可视化参数
  bool debug_enable_csv_dump;           ///< 启用CSV文件导出
  std::string debug_csv_output_dir;     ///< CSV导出目录
  double debug_trajectory_sample_dt;    ///< 轨迹采样步长 (s)
  bool debug_enable_rviz_publish;       ///< 启用ROS可视化发布

  /// 默认构造函数
  UseCaseConfig()
    : record_delay_s(15.0),
      record_max_hz(10.0),
      record_min_points(100),
      debug_enable_csv_dump(false),
      debug_csv_output_dir(""),
      debug_trajectory_sample_dt(0.05),
      debug_enable_rviz_publish(false)
  {}
};

// ============================================================================
// Tick输出数据
// ============================================================================

/// UseCase tick输出数据
struct TickOutput {
  bool should_publish_setpoint;                    ///< 是否应发布setpoint
  std::optional<domain::SetpointTarget> setpoint;  ///< setpoint目标
  std::optional<domain::ModeRequest> mode_request; ///< 模式切换请求
  domain::State current_state;                     ///< 当前状态（用于调试）
  bool rc_loss;                                    ///< RC丢失持续状态
  bool low_battery;                                ///< 低电量持续状态

  /// 默认构造函数
  TickOutput()
    : should_publish_setpoint(false),
      current_state(domain::State::IdleDisarmed),
      rc_loss(false),
      low_battery(false)
  {}
};

// ============================================================================
// BacktrackUseCase类
// ============================================================================

/// 返航用例（应用层）
class BacktrackUseCase {
public:
  /// 构造函数
  /// @param config 配置参数
  explicit BacktrackUseCase(const UseCaseConfig& config);

  /// 每个20Hz周期调用的tick
  /// @param now 当前时间 (s)
  /// @return tick输出
  TickOutput tick(double now);

  /// 里程计回调
  /// @param position 位置 (ENU, m)
  /// @param velocity 线速度 (ENU, m/s)
  /// @param yaw Yaw角 (rad)
  /// @param stamp 时间戳 (s)
  void onOdomReceived(const domain::Vec3& position,
                      const domain::Vec3& velocity,
                      double yaw,
                      double stamp);

  /// RC回调
  /// @param channels RC通道数组
  void onRCReceived(const std::vector<uint16_t>& channels);

  /// 飞控状态回调
  /// @param armed 是否ARM
  /// @param mode 当前模式
  void onStateReceived(bool armed, const std::string& mode);

  /// 电池状态回调
  /// @param percentage 电量百分比 (0.0~1.0)
  void onBatteryReceived(double percentage);

  /// 设置 CSV debug dumper（可选）
  void setDebugDumper(std::shared_ptr<infrastructure::DebugDumper> dumper);

  /// 设置 ROS debug publisher（可选）
  void setDebugPublisher(std::shared_ptr<adapters::DebugPublisher> publisher);

private:
  /// 配置参数
  UseCaseConfig config_;

  // Domain对象
  domain::RecordingBuffer recording_buffer_;
  domain::TriggerEvaluator trigger_evaluator_;
  domain::TrajectorySession trajectory_session_;
  domain::WaypointFallbackSession waypoint_fallback_session_;
  domain::RejoinPolicy rejoin_policy_;
  domain::BacktrackStateMachine state_machine_;

  // 当前状态快照
  bool current_armed_;
  std::string current_mode_;
  domain::Vec3 current_position_;
  domain::Vec3 current_velocity_;
  double current_yaw_;

  // 飞行周期复位检测
  bool has_seen_disarm_;
  bool arm_after_disarm_detected_;

  // 录制延时
  double record_delay_start_time_;
  bool record_delay_started_;

  // 轨迹生成标志
  bool trajectory_generation_requested_;

  double last_progress_time_ = -1.0;

  // Rejoin持续判定计时
  double rejoin_trigger_candidate_start_time_;
  double rejoin_success_candidate_start_time_;

  // 返航前起始航向对准计时
  double yaw_alignment_start_time_;
  bool yaw_alignment_timeout_reported_;

  // 上周期已发布的路径目标速度，用于 Rejoin 速度连续性
  domain::Vec3 last_path_target_velocity_;
  bool has_last_path_target_velocity_;

  // AUTO.LAND让步锁存：一旦外部降落接管，直到DISARM前都不再触发返航
  bool auto_land_yield_active_;

  /// 飞行周期复位检测
  void handleFlightCycleReset(bool armed);

  /// AUTO.LAND让步处理
  bool handleAutoLandYield();

  /// 录制延时处理
  void handleRecordDelay(double now);

  /// 触发检测与处理
  void handleTriggers(double now);

  /// 根据点数条件执行返航或直接降落
  void handleReturnTrigger(domain::Event event,
                           bool has_min_points,
                           const char* trigger_name);

  /// 轨迹生成处理
  void handleTrajectoryGeneration(double now);

  /// 退化 waypoint 会话执行进度处理
  void handleWaypointFallbackProgress();

  /// 按实际位置更新弧长路径进度
  void handleTrajectoryProgress(double now);

  /// 轨迹是否已完成起始航向对准，可进入 ReturnActive
  bool isTrajectoryActivationReady(double now);

  /// 偏离检测与回归处理
  void handleRejoin(double now);

  /// 计算setpoint输出
  domain::SetpointTarget computeSetpoint();

  /// 检查点数是否足够
  bool checkMinPoints() const;

  /// 统一处理“不可返航，直接降落”的降级路径
  void triggerImmediateLanding();

  /// 执行 debug 数据导出（轨迹生成成功后调用）
  void performDebugDump();

  // Debug helpers（可选，nullptr 表示禁用）
  std::shared_ptr<infrastructure::DebugDumper> debug_dumper_;
  std::shared_ptr<adapters::DebugPublisher> debug_publisher_;
};

}  // namespace application
}  // namespace px4_backtrack
