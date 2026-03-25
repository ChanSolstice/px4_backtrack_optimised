#include <gtest/gtest.h>

#include "px4_backtrack/domain/state_machine.hpp"

namespace px4_backtrack::domain {
namespace {

TickInput makeInput(bool armed,
                    const std::string& mode,
                    double now,
                    bool trajectory_ready = false,
                    bool trajectory_finished = false) {
  TickInput input;
  input.armed = armed;
  input.mode = mode;
  input.now = now;
  input.current_position = Vec3(1.0, 2.0, 3.0);
  input.current_yaw = 0.4;
  input.trajectory_ready = trajectory_ready;
  input.trajectory_finished = trajectory_finished;
  return input;
}

TEST(StateMachineTest, ArmThenStartRecordingTransitionsIntoRecording) {
  BacktrackStateMachine machine{StateMachineConfig()};

  machine.tick(makeInput(false, "", 0.0));
  EXPECT_EQ(machine.getCurrentState(), State::IdleDisarmed);

  machine.tick(makeInput(true, "", 0.1));
  EXPECT_EQ(machine.getCurrentState(), State::WaitRecordDelay);

  machine.startRecording();
  EXPECT_EQ(machine.getCurrentState(), State::Recording);
}

TEST(StateMachineTest, ReturnPreOffboardRequestsOffboardAndEntersReturnActive) {
  StateMachineConfig config;
  config.pre_offboard_setpoint_count = 2;
  config.offboard_attempt_interval_s = 0.0;

  BacktrackStateMachine machine(config);

  machine.tick(makeInput(true, "", 0.0));
  machine.startRecording();
  machine.lockPosition(Vec3(3.0, 4.0, 5.0), 1.2);
  machine.triggerReturn(Event::Ch7LongPress);

  machine.tick(makeInput(true, "", 0.1));
  EXPECT_EQ(machine.getCurrentState(), State::ReturnPreOffboard);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
  ASSERT_TRUE(machine.getSetpointTarget().has_value());
  EXPECT_TRUE(machine.getSetpointTarget()->position.isApprox(Vec3(3.0, 4.0, 5.0)));
  EXPECT_FALSE(machine.getModeRequest().has_value());

  machine.tick(makeInput(true, "", 0.2));
  EXPECT_FALSE(machine.getModeRequest().has_value());

  machine.tick(makeInput(true, "", 0.3));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::OFFBOARD);

  machine.tick(makeInput(true, "OFFBOARD", 0.4, true));
  EXPECT_EQ(machine.getCurrentState(), State::ReturnActive);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
}

TEST(StateMachineTest, AutoLandHasGlobalPriorityAndYieldsControl) {
  StateMachineConfig config;
  config.pre_offboard_setpoint_count = 0;

  BacktrackStateMachine machine(config);
  machine.tick(makeInput(true, "", 0.0));
  machine.startRecording();
  machine.lockPosition(Vec3(0.0, 0.0, 1.0), 0.0);
  machine.triggerReturn(Event::Ch7LongPress);
  machine.tick(makeInput(true, "OFFBOARD", 0.1, true));
  ASSERT_EQ(machine.getCurrentState(), State::ReturnActive);

  machine.tick(makeInput(true, "AUTO.LAND", 0.2));

  EXPECT_EQ(machine.getCurrentState(), State::Recording);
  EXPECT_FALSE(machine.shouldPublishSetpoint());
  EXPECT_FALSE(machine.getModeRequest().has_value());
}

TEST(StateMachineTest, DirectLandingRequestsAutoLandImmediatelyWhenNotInOffboard) {
  BacktrackStateMachine machine{StateMachineConfig()};

  machine.tick(makeInput(true, "POSCTL", 0.0));
  machine.startRecording();
  machine.triggerLanding();
  machine.tick(makeInput(true, "POSCTL", 0.1));

  EXPECT_EQ(machine.getCurrentState(), State::LandingTransition);
  EXPECT_FALSE(machine.shouldPublishSetpoint());
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::AUTO_LAND);
}

}  // namespace
}  // namespace px4_backtrack::domain
