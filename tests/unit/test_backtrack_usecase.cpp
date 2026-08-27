#include <gtest/gtest.h>

#include "px4_backtrack/application/backtrack_usecase.hpp"

namespace px4_backtrack::application {
namespace {

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

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.2);

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

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), 0.0, 1.2);

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

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), 0.0, 1.2);

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
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  // AUTO.LAND一旦接管，即使模式短暂跳出且触发通道再次动作，也不能重新请求OFFBOARD。
  usecase.onStateReceived(true, "POSCTL");
  usecase.onRCReceived(makeRcChannels(1500));
  output = usecase.tick(0.6);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  usecase.onRCReceived(makeRcChannels(1000));
  output = usecase.tick(0.7);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  output = usecase.tick(0.8);
  EXPECT_EQ(output.current_state, domain::State::Recording);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());

  usecase.onStateReceived(false, "AUTO.LAND");
  output = usecase.tick(0.9);
  EXPECT_EQ(output.current_state, domain::State::IdleDisarmed);
  EXPECT_FALSE(output.should_publish_setpoint);
  EXPECT_FALSE(output.mode_request.has_value());
}

TEST(BacktrackUseCaseTest, DoesNotFinishTimedTrajectoryBeforeReachingReturnGoal) {
  BacktrackUseCase usecase(makeConfig());

  usecase.onStateReceived(false, "");
  usecase.tick(0.0);
  usecase.onStateReceived(true, "OFFBOARD");
  usecase.tick(0.1);
  usecase.tick(0.2);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 1.0);
  usecase.onOdomReceived(domain::Vec3(1.0, 0.0, 1.0), 0.0, 1.1);
  usecase.onOdomReceived(domain::Vec3(2.0, 0.0, 1.0), 0.0, 1.2);

  usecase.onRCReceived(makeRcChannels(1000));
  usecase.tick(0.3);
  auto output = usecase.tick(0.4);
  ASSERT_EQ(output.current_state, domain::State::ReturnActive);

  // ReturnPreOffboard切换周期仍会保留一次hold setpoint，先启动轨迹时钟。
  usecase.tick(10000.0);
  output = usecase.tick(20000.0);
  EXPECT_EQ(output.current_state, domain::State::ReturnActive);
  EXPECT_TRUE(output.should_publish_setpoint);

  usecase.onOdomReceived(domain::Vec3(0.0, 0.0, 1.0), 0.0, 20000.1);
  output = usecase.tick(20000.1);
  EXPECT_EQ(output.current_state, domain::State::LandingHover);
  EXPECT_TRUE(output.should_publish_setpoint);
}

}  // namespace
}  // namespace px4_backtrack::application
