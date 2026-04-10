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

}  // namespace
}  // namespace px4_backtrack::application
