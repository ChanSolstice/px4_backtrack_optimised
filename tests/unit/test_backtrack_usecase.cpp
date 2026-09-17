#include <gtest/gtest.h>

#include <limits>

#include "px4_backtrack/application/backtrack_usecase.hpp"

namespace px4_backtrack::application {
namespace {

const domain::Vec3 kZeroVelocity = domain::Vec3::Zero();
constexpr double kPi = 3.14159265358979323846;

UseCaseConfig makeConfig() {
  UseCaseConfig config;
  config.record_delay_s = 0.0;
  config.record_max_hz = 100.0;
  config.record_min_points = 3;
  config.state_machine_config.pre_offboard_setpoint_count = 0;
  config.state_machine_config.offboard_attempt_interval_s = 0.0;

  config.trigger_config.enable_ch7 = true;
  config.trigger_config.ch7_pressed_threshold = 1200;
  config.trigger_config.ch7_hold_s = 0.0;
  config.trigger_config.enable_ch18 = false;
  config.trigger_config.enable_rc_loss = false;
  config.trigger_config.enable_low_battery = false;
  config.trigger_config.low_battery_threshold = 0.15;
  config.trigger_config.low_battery_confirm_s = 0.0;

  config.trajectory_config.ds_m = 0.1;
  config.trajectory_config.min_point_dist_m = 0.01;

  return config;
}

std::vector<uint16_t> makeRcChannels(uint16_t ch7) {
  std::vector<uint16_t> channels(18, 1500);
  channels[6] = ch7;
  return channels;
}

TEST(BacktrackUseCaseTest, FallsBackToImmediateLandingWhenTrajectoryGenerationFails) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  auto output = usecase.tick(0.0);
  EXPECT_EQ(output.current_state, domain::State::IdleDisarmed);

  usecase.onStateReceived(true, "POSCTL");
  output = usecase.tick(0.1);
  EXPECT_EQ(output.current_state, domain::State::WaitRecordDelay);

  output = usecase.tick(0.2);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.3);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  output = usecase.tick(0.4);
  EXPECT_EQ(output.current_state, domain::State::LandingTransition);
  EXPECT_FALSE(output.should_publish_setpoint);
  ASSERT_TRUE(output.mode_request.has_value());
  EXPECT_EQ(*output.mode_request, domain::ModeRequest::AUTO_LAND);
}

TEST(BacktrackUseCaseTest, DoesNotFallbackWhenTrajectoryConfigurationIsInvalid) {
  auto config = makeConfig();
  config.trajectory_config.v_end_max = config.trajectory_config.v_max + 1.0;

  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  auto output = usecase.tick(0.0);
  EXPECT_EQ(output.current_state, domain::State::IdleDisarmed);

  usecase.onStateReceived(true, "OFFBOARD");
  output = usecase.tick(0.1);
  EXPECT_EQ(output.current_state, domain::State::WaitRecordDelay);

  output = usecase.tick(0.2);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.3);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  output = usecase.tick(0.4);
  EXPECT_EQ(output.current_state, domain::State::ReturnPreOffboard);
  EXPECT_TRUE(output.should_publish_setpoint);
  ASSERT_TRUE(output.setpoint.has_value());
  EXPECT_NEAR(output.setpoint->position.x(), 2.0, 1e-6);
  EXPECT_FALSE(output.mode_request.has_value());
}

TEST(BacktrackUseCaseTest, AutoLandYieldSuppressesRetriggerUntilDisarm) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  auto output = usecase.tick(0.0);
  EXPECT_EQ(output.current_state, domain::State::IdleDisarmed);

  usecase.onStateReceived(true, "OFFBOARD");
  output = usecase.tick(0.1);
  EXPECT_EQ(output.current_state, domain::State::WaitRecordDelay);

  output = usecase.tick(0.2);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.3);
  EXPECT_EQ(output.current_state, domain::State::Recording);

  output = usecase.tick(0.4);
  ASSERT_EQ(output.current_state, domain::State::ReturnActive);
  ASSERT_TRUE(output.setpoint.has_value());
  EXPECT_EQ(output.setpoint->type_mask,
            domain::position_target_mask::kIgnoreAcceleration);
  EXPECT_FALSE(output.mode_request.has_value());

  usecase.onStateReceived(true, "AUTO.LAND");
  output = usecase.tick(0.5);
  EXPECT_EQ(output.current_state, domain::State::AutoLandYield);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  // AUTO.LAND一旦接管，即使模式短暂跳出且触发通道再次动作，也不能重新请求OFFBOARD。
  usecase.onStateReceived(true, "POSCTL");
  usecase.onRCReceived(makeRcChannels(1500));
  output = usecase.tick(0.6);
  EXPECT_EQ(output.current_state, domain::State::AutoLandYield);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.7);
  EXPECT_EQ(output.current_state, domain::State::AutoLandYield);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  output = usecase.tick(0.8);
  EXPECT_EQ(output.current_state, domain::State::AutoLandYield);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  usecase.onStateReceived(false, "AUTO.LAND");
  output = usecase.tick(0.9);
  EXPECT_EQ(output.current_state, domain::State::IdleDisarmed);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());
}

TEST(BacktrackUseCaseTest, WallTimeCannotAdvanceSpatialTrajectory) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  auto output = usecase.tick(0.4);
  ASSERT_EQ(output.current_state, domain::State::ReturnActive);

  // 无论 wall time 跳变多大，实际位置未前进时空间进度都不会结束。
  output = usecase.tick(20000.0);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);
  EXPECT_TRUE(output.should_publish_setpoint);

  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, kPi, 20000.1);
  output = usecase.tick(20000.1);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, kPi, 20000.2);
  output = usecase.tick(20000.2);
  EXPECT_EQ(output.current_state, domain::State::LandingHover);
  EXPECT_TRUE(output.should_publish_setpoint);
}

TEST(BacktrackUseCaseTest, ReturnEndToleranceIsIndependentFromRejoinSuccessDistance) {
  auto config = makeConfig();
  config.rejoin_config.success_dist_m = 0.4;
  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);
  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  ASSERT_EQ(usecase.tick(0.4).current_state, domain::State::ReturnActive);

  // 回到 Rejoin 的 0.4m 安全走廊不等于到达终点，终点仍要求 0.2m。
  usecase.onOdomReceived(
      domain::Vec3(0.0, 0.0, 1.3), kZeroVelocity, kPi, 1.3);
  auto output = usecase.tick(0.5);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);

  usecase.onOdomReceived(
      domain::Vec3(0.0, 0.0, 1.19), kZeroVelocity, kPi, 1.4);
  output = usecase.tick(0.6);
  EXPECT_EQ(output.current_state, domain::State::LandingHover);
}

TEST(BacktrackUseCaseTest, AlignsStartYawBeforeHorizontalReturn) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  auto output = usecase.tick(0.4);

  ASSERT_EQ(output.current_state, domain::State::ReturnPreOffboard);
  ASSERT_TRUE(output.setpoint.has_value());
  EXPECT_TRUE(output.setpoint->position.isApprox(domain::Vec3(2.0, 0.0, 1.0)));
  EXPECT_TRUE(output.setpoint->velocity.isApprox(kZeroVelocity));
  EXPECT_NEAR(output.setpoint->yaw, kPi, 1e-3);

  // 对准超时也不允许带着大航向误差开始平移。
  output = usecase.tick(20.0);
  EXPECT_EQ(output.current_state, domain::State::ReturnPreOffboard);
  ASSERT_TRUE(output.setpoint.has_value());
  EXPECT_TRUE(output.setpoint->velocity.isApprox(kZeroVelocity));

  usecase.onOdomReceived(
      domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 20.1);
  output = usecase.tick(20.1);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);
}

TEST(BacktrackUseCaseTest, RejoinUsesPathYawAndWaitsForVelocityConvergence) {
  auto config = makeConfig();
  config.rejoin_config.trigger_dist_m = 0.3;
  config.rejoin_config.success_dist_m = 0.1;
  config.rejoin_config.trigger_hold_s = 0.0;
  config.rejoin_config.success_hold_s = 0.0;
  config.trajectory_config.enable_end_converge = false;
  config.trajectory_config.v_max = 1.0;
  config.trajectory_config.v_end_max = 1.0;
  config.trajectory_config.v_start_m_s = 1.0;
  config.trajectory_config.v_end_m_s = 1.0;
  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);
  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  auto active_output = usecase.tick(0.4);
  ASSERT_EQ(active_output.current_state, domain::State::ReturnActive);
  ASSERT_TRUE(active_output.setpoint.has_value());

  usecase.onOdomReceived(
      domain::Vec3(1.9, 0.8, 1.0), kZeroVelocity, kPi, 1.3);
  auto rejoin_output = usecase.tick(0.5);
  ASSERT_EQ(rejoin_output.current_state, domain::State::ReturnRejoin);
  ASSERT_TRUE(rejoin_output.setpoint.has_value());
  EXPECT_NEAR(rejoin_output.setpoint->yaw, kPi, 1e-3);
  EXPECT_LE((rejoin_output.setpoint->velocity -
             active_output.setpoint->velocity).norm(),
            0.3 + 1e-9);

  // 位置已回到路径但速度尚未恢复时，不能退出 Rejoin。
  usecase.onOdomReceived(
      domain::Vec3(1.8, 0.0, 1.0), kZeroVelocity, kPi, 1.4);
  rejoin_output = usecase.tick(0.6);
  EXPECT_EQ(rejoin_output.current_state, domain::State::ReturnRejoin);

  usecase.onOdomReceived(
      domain::Vec3(1.8, 0.0, 1.0), domain::Vec3(-1.0, 0.0, 0.0), kPi, 1.5);
  auto recovered_output = usecase.tick(0.7);
  EXPECT_EQ(recovered_output.current_state, domain::State::ReturnActive);
  ASSERT_TRUE(recovered_output.setpoint.has_value());
  EXPECT_NEAR(recovered_output.setpoint->yaw, kPi, 1e-3);
  EXPECT_LE((recovered_output.setpoint->velocity -
             rejoin_output.setpoint->velocity).norm(),
            0.3 + 1e-9);
  EXPECT_TRUE(recovered_output.setpoint->position.isApprox(
      rejoin_output.setpoint->position, 1e-6));
}

TEST(BacktrackUseCaseTest, ResumeAlignsToRetainedPathTargetYaw) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);
  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  ASSERT_EQ(usecase.tick(0.4).current_state, domain::State::ReturnActive);

  usecase.onRCReceived(makeRcChannels(1500));
  EXPECT_EQ(usecase.tick(0.5).current_state, domain::State::ReturnActive);
  usecase.onRCReceived(makeRcChannels(1000));
  EXPECT_EQ(usecase.tick(0.6).current_state, domain::State::Paused);

  usecase.onRCReceived(makeRcChannels(1500));
  EXPECT_EQ(usecase.tick(0.7).current_state, domain::State::Paused);
  usecase.onOdomReceived(
      domain::Vec3(1.9, 0.0, 1.0), kZeroVelocity, 0.0, 1.3);
  usecase.onRCReceived(makeRcChannels(1000));
  EXPECT_EQ(usecase.tick(0.8).current_state, domain::State::Paused);
  auto output = usecase.tick(0.9);

  ASSERT_EQ(output.current_state, domain::State::ReturnPreOffboard);
  ASSERT_TRUE(output.setpoint.has_value());
  EXPECT_TRUE(output.setpoint->position.isApprox(domain::Vec3(1.9, 0.0, 1.0)));
  EXPECT_TRUE(output.setpoint->velocity.isApprox(kZeroVelocity));
  EXPECT_NEAR(output.setpoint->yaw, kPi, 1e-3);

  usecase.onOdomReceived(
      domain::Vec3(1.9, 0.0, 1.0), kZeroVelocity, kPi, 1.4);
  EXPECT_EQ(usecase.tick(1.0).current_state, domain::State::ReturnActive);
}

TEST(BacktrackUseCaseTest, ProjectionFailureEntersRejoinAndKeepsLastTarget) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);
  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  const auto initial_output = usecase.tick(0.4);
  ASSERT_EQ(initial_output.current_state, domain::State::ReturnActive);
  ASSERT_TRUE(initial_output.setpoint.has_value());

  domain::Vec3 invalid_position = domain::Vec3::Zero();
  invalid_position.x() = std::numeric_limits<double>::quiet_NaN();
  usecase.onOdomReceived(invalid_position, kZeroVelocity, kPi, 1.3);
  const auto failed_output = usecase.tick(0.5);

  EXPECT_EQ(failed_output.current_state, domain::State::ReturnRejoin);
  ASSERT_TRUE(failed_output.setpoint.has_value());
  EXPECT_TRUE(failed_output.setpoint->position.isApprox(
      initial_output.setpoint->position));
  EXPECT_FALSE(failed_output.setpoint->position.isApprox(domain::Vec3::Zero()));
}

TEST(BacktrackUseCaseTest, LowBatteryTriggersReturnDuringRecording) {
  auto config = makeConfig();
  config.trigger_config.enable_ch7 = false;
  config.trigger_config.enable_low_battery = true;
  config.trigger_config.low_battery_threshold = 0.15;
  config.trigger_config.low_battery_confirm_s = 0.5;

  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  auto output = usecase.tick(0.2);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.low_battery);

  // 记录足够点数
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);

  // 收到低电量消息
  usecase.onBatteryReceived(0.10);
  output = usecase.tick(0.3);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.low_battery);

  // 确认延时未达到
  output = usecase.tick(0.5);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.low_battery);

  // 达到确认延时 (0.8 - 0.3 = 0.5s)
  output = usecase.tick(0.8);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);
  EXPECT_TRUE(output.low_battery);
  EXPECT_TRUE(output.should_publish_setpoint);
}

TEST(BacktrackUseCaseTest, LowBatteryFallsBackToLandingWithInsufficientPoints) {
  auto config = makeConfig();
  config.record_min_points = 10;
  config.trigger_config.enable_ch7 = false;
  config.trigger_config.enable_low_battery = true;
  config.trigger_config.low_battery_threshold = 0.15;
  config.trigger_config.low_battery_confirm_s = 0.0;

  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "POSCTL");
  usecase.tick(0.1);
  usecase.tick(0.2);

  // 只记录2个点 (< 10)
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);

  usecase.onBatteryReceived(0.08);
  usecase.tick(0.3);
  auto output = usecase.tick(0.4);

  // 点数不足降级为直接降落
  EXPECT_EQ(output.current_state, domain::State::LandingTransition);
  EXPECT_TRUE(output.low_battery);
  EXPECT_FALSE(output.should_publish_setpoint);
  ASSERT_TRUE(output.mode_request.has_value());
  EXPECT_EQ(*output.mode_request, domain::ModeRequest::AUTO_LAND);
}

TEST(BacktrackUseCaseTest, LowBatteryInPausedBroadcastsButDoesNotResume) {
  auto config = makeConfig();
  config.trigger_config.enable_ch7 = true;
  config.trigger_config.enable_low_battery = true;
  config.trigger_config.low_battery_threshold = 0.15;
  config.trigger_config.low_battery_confirm_s = 0.0;

  BacktrackUseCase usecase(config);

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), kZeroVelocity, 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), kZeroVelocity, kPi, 1.2);

  // CH7长按触发返航
  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  auto output = usecase.tick(0.4);
  ASSERT_EQ(output.current_state, domain::State::ReturnActive);

  // 释放CH7，再单击暂停
  usecase.onRCReceived(makeRcChannels(1500));
  usecase.tick(0.5);
  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.6);
  ASSERT_EQ(output.current_state, domain::State::Paused);
  EXPECT_FALSE(output.low_battery);

  // 释放按键，避免长按逻辑再次触发恢复
  usecase.onRCReceived(makeRcChannels(1500));

  // 在 Paused 状态下收到低电量报警
  usecase.onBatteryReceived(0.09);
  output = usecase.tick(0.7);
  // 核心业务设计：Paused状态下不自动恢复返航，但广播低电量状态供前端弹窗
  EXPECT_EQ(output.current_state, domain::State::Paused);
  EXPECT_TRUE(output.low_battery);

  output = usecase.tick(1.0);
  EXPECT_EQ(output.current_state, domain::State::Paused);
  EXPECT_TRUE(output.low_battery);
}

}  // namespace
}  // namespace px4_backtrack::application
