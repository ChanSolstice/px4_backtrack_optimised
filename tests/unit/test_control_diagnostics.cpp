#include <gtest/gtest.h>
#include "px4_backtrack/application/backtrack_usecase.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace px4_backtrack::application {
namespace {

UseCaseConfig diagnosticsConfig() {
  UseCaseConfig c;
  c.record_delay_s = 0.0;
  c.record_min_points = 3;
  c.record_max_hz = 100.0;
  c.state_machine_config.pre_offboard_setpoint_count = 0;
  c.state_machine_config.offboard_attempt_interval_s = 0.0;
  c.trigger_config.enable_ch7 = true;
  c.trigger_config.ch7_hold_s = 0.0;
  c.trigger_config.ch7_pressed_threshold = 1200;
  c.trigger_config.enable_ch18 = false;
  c.trigger_config.enable_rc_loss = false;
  c.trigger_config.enable_low_battery = false;
  return c;
}

struct DiagnosticFlight {
  BacktrackUseCase usecase;
  double now = 10.0, stamp = 10.0;
  bool armed = false;
  std::string mode = "OFFBOARD";
  domain::Vec3 position{0, 0, 1}, velocity = domain::Vec3::Zero();
  double yaw = std::acos(-1.0);
  TickOutput output;

  explicit DiagnosticFlight(UseCaseConfig c = diagnosticsConfig()) : usecase(c) {
    step();
    armed = true;
    step();
    step();
  }
  TickOutput step(double dt = 0.05, bool odom = true, bool state = true) {
    now += dt;
    stamp += dt;
    if (state) usecase.onStateReceived(armed, mode, now);
    if (odom) usecase.onOdomReceived(position, velocity, yaw, stamp, now);
    output = usecase.tick(now);
    return output;
  }
  ControlDiagnostics snapshot() const { return usecase.diagnostics(now, output); }
  void rc(uint16_t channel) {
    std::vector<uint16_t> channels(18, 1500);
    channels[6] = channel;
    usecase.onRCReceived(channels, now);
  }
  void startReturn() {
    for (int i = 0; i <= 20; ++i) {
      position.x() = i * 0.1;
      step(0.2);
    }
    rc(1000);
    step();
    step();
    for (int i = 0; i < 2000 && output.current_state == domain::State::ReturnPreOffboard; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      step();
    }
  }
};

TEST(ControlDiagnosticsTest, MissingAndNonfiniteInputEarlyReturnsAreObservable) {
  BacktrackUseCase usecase(diagnosticsConfig());
  auto out = usecase.tick(1.0);
  auto d = usecase.diagnostics(1.0, out);
  EXPECT_EQ(d.schema_version, 1U);
  EXPECT_FALSE(d.inputs_fresh);
  EXPECT_EQ(d.input_failure_reason, "odometry_missing");
  EXPECT_DOUBLE_EQ(d.odom_receipt_age, -1.0);
  EXPECT_DOUBLE_EQ(d.state_receipt_age, -1.0);
  EXPECT_FALSE(d.setpoint);
  EXPECT_EQ(d.current_state, out.current_state);
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  out = usecase.tick(invalid);
  d = usecase.diagnostics(invalid, out);
  EXPECT_FALSE(d.inputs_fresh);
  EXPECT_EQ(d.input_failure_reason, "nonfinite_tick_clock");
  EXPECT_EQ(d.operation, "tick_nonfinite_clock");
}

TEST(ControlDiagnosticsTest, FlightIdentifierTracksAcceptedArmEdgesOnly) {
  DiagnosticFlight f;
  EXPECT_EQ(f.snapshot().flight_id, 1U);
  f.step();
  f.step();
  EXPECT_EQ(f.snapshot().flight_id, 1U);
  f.usecase.onStateReceived(false, "POSCTL", f.now - 1.0); // rejected old receipt
  f.step();
  EXPECT_EQ(f.snapshot().flight_id, 1U);
  f.armed = false;
  f.step();
  EXPECT_EQ(f.snapshot().flight_id, 1U);
  f.armed = true;
  f.step();
  EXPECT_EQ(f.snapshot().flight_id, 2U);
}

TEST(ControlDiagnosticsTest, OdometryRejectionRetainsLastAcceptedPoseAndResetReason) {
  DiagnosticFlight f;
  const auto accepted = f.snapshot();
  f.usecase.onOdomReceived(domain::Vec3::Constant(std::numeric_limits<double>::quiet_NaN()),
                          f.velocity, f.yaw, f.stamp + 0.01, f.now + 0.01);
  f.step(0.05, false);
  auto d = f.snapshot();
  EXPECT_FALSE(d.odom_valid);
  EXPECT_EQ(d.input_failure_reason, "odometry_invalid");
  EXPECT_EQ(d.odometry_rejection_reason, "nonfinite_or_negative_values");
  EXPECT_TRUE(d.actual_position.isApprox(accepted.actual_position));
  f.step();
  EXPECT_TRUE(f.snapshot().odom_valid);
  EXPECT_TRUE(f.snapshot().odometry_rejection_reason.empty());
  f.stamp -= 2.0;
  f.step();
  d = f.snapshot();
  EXPECT_TRUE(d.localization_reset);
  EXPECT_EQ(d.input_failure_reason, "localization_reset");
  EXPECT_EQ(d.odometry_rejection_reason, "stamp_moved_backward");
}

TEST(ControlDiagnosticsTest, StaleInputHoldCapturesExactOutputAndDoesNotRepeatOperation) {
  DiagnosticFlight f;
  f.startReturn();
  ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
  f.step(0.30, false);
  auto d = f.snapshot();
  EXPECT_EQ(d.current_state, domain::State::Paused);
  EXPECT_EQ(d.input_failure_reason, "odometry_stale");
  EXPECT_TRUE(d.input_hold);
  EXPECT_EQ(d.operation, "input_hold:odometry_stale");
  EXPECT_EQ(d.mode_request, domain::ModeRequest::POSCTL);
  ASSERT_TRUE(d.setpoint);
  EXPECT_TRUE(d.should_publish_setpoint);
  EXPECT_EQ(d.setpoint->type_mask, 0x09F8);
  EXPECT_TRUE(d.setpoint->position.isApprox(f.output.setpoint->position));
  EXPECT_DOUBLE_EQ(d.setpoint->velocity.norm(), 0.0);
  const auto sequence = d.operation_sequence;
  f.step(0.05, false);
  EXPECT_EQ(f.snapshot().operation_sequence, sequence);
  // 相同坏帧持续到来时也不反复生成 operation；原始输入由 rosbag 保留。
  f.usecase.onOdomReceived(domain::Vec3::Constant(std::numeric_limits<double>::quiet_NaN()),
                          f.velocity, f.yaw, f.stamp + 0.01, f.now + 0.01);
  f.step(0.05, false);
  const auto rejected_sequence = f.snapshot().operation_sequence;
  for (int i = 0; i < 10; ++i) {
    f.usecase.onOdomReceived(domain::Vec3::Constant(std::numeric_limits<double>::quiet_NaN()),
                            f.velocity, f.yaw, f.stamp + 0.01, f.now + 0.01);
    f.step(0.05, false);
  }
  EXPECT_EQ(f.snapshot().operation_sequence, rejected_sequence);
}

TEST(ControlDiagnosticsTest, ExternalControlYieldIsRecordedBeforeInputEarlyReturn) {
  DiagnosticFlight f;
  f.startReturn();
  ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
  f.mode = "POSCTL";
  f.usecase.onOdomReceived(domain::Vec3::Constant(std::numeric_limits<double>::quiet_NaN()),
                          f.velocity, f.yaw, f.stamp + 0.01, f.now + 0.01);
  f.step(0.05, false);
  const auto d = f.snapshot();
  EXPECT_EQ(d.current_state, domain::State::ControlYield);
  EXPECT_EQ(d.operation, "external_offboard_exit_yield");
  EXPECT_FALSE(d.should_publish_setpoint);
  EXPECT_FALSE(d.setpoint);
  EXPECT_EQ(d.fcu_mode, "POSCTL");
}

TEST(ControlDiagnosticsTest, AutoLandEarlyReturnPreservesYieldReason) {
  DiagnosticFlight f;
  f.startReturn();
  ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
  f.mode = "AUTO.LAND";
  f.step(0.05, false);
  const auto d = f.snapshot();
  EXPECT_EQ(d.current_state, domain::State::AutoLandYield);
  EXPECT_TRUE(d.auto_land_yield);
  EXPECT_EQ(d.operation, "auto_land_yield");
  EXPECT_FALSE(d.setpoint);
  EXPECT_FALSE(d.mode_request);
}

TEST(ControlDiagnosticsTest, RouteSafetyCachesPredictionAndDocumentsHold) {
  DiagnosticFlight f;
  f.startReturn();
  ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
  f.position.z() += 0.20;
  f.velocity.z() = 0.30;
  f.step();
  auto d = f.snapshot();
  EXPECT_TRUE(d.route_projection_valid);
  EXPECT_TRUE(d.route_prediction_valid);
  EXPECT_NEAR(d.route_distance, 0.20, 1e-9);
  EXPECT_GE(d.route_predicted_deviation, 0.30);
  EXPECT_DOUBLE_EQ(d.route_safety_evaluation_time, f.now);
  EXPECT_TRUE(d.route_safety_hold);
  EXPECT_EQ(d.operation, "route_deviation_budget_hold");
  const auto before = d;
  for (int i = 0; i < 50; ++i) d = f.snapshot();
  EXPECT_DOUBLE_EQ(d.route_predicted_deviation, before.route_predicted_deviation);
  EXPECT_DOUBLE_EQ(d.route_distance, before.route_distance);
  EXPECT_EQ(d.operation_sequence, before.operation_sequence);
}

TEST(ControlDiagnosticsTest, SnapshotDoesNotAdvanceClockProjectionOrFallback) {
  for (bool fallback : {false, true}) {
    auto c = diagnosticsConfig();
    if (fallback) c.trajectory_config.v_end_max = 2.0;
    DiagnosticFlight f(c);
    f.startReturn();
    ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
    const auto before = f.snapshot();
    EXPECT_EQ(before.flight_id, 1U);
    EXPECT_EQ(before.plan_id, 1U);
    EXPECT_EQ(before.last_trigger, "Ch7LongPress");
    EXPECT_EQ(before.fallback_valid, fallback);
    EXPECT_EQ(before.trajectory_valid, !fallback);
    for (int i = 0; i < 1000; ++i) {
      const auto after = f.usecase.diagnostics(f.now + 10.0, f.output);
      EXPECT_DOUBLE_EQ(after.reference_time, before.reference_time);
      EXPECT_DOUBLE_EQ(after.clock_rate, before.clock_rate);
      EXPECT_DOUBLE_EQ(after.projected_s, before.projected_s);
      EXPECT_DOUBLE_EQ(after.target_s, before.target_s);
      EXPECT_EQ(after.fallback_index, before.fallback_index);
      EXPECT_EQ(after.operation_sequence, before.operation_sequence);
      EXPECT_EQ(after.current_state, before.current_state);
    }
  }
}

TEST(ControlDiagnosticsTest, BackwardTickClockIsDistinguishableFromInputTimeout) {
  DiagnosticFlight f;
  f.startReturn();
  ASSERT_EQ(f.output.current_state, domain::State::ReturnActive);
  f.now -= 0.10;
  f.output = f.usecase.tick(f.now);
  const auto d = f.snapshot();
  EXPECT_TRUE(d.tick_clock_reset);
  EXPECT_FALSE(d.inputs_fresh);
  EXPECT_EQ(d.input_failure_reason, "tick_clock_moved_backward");
  EXPECT_EQ(d.operation, "input_hold:tick_clock_moved_backward");
}

} // namespace
} // namespace px4_backtrack::application
