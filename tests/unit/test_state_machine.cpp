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
  EXPECT_EQ(machine.getSetpointTarget()->type_mask,
            position_target_mask::kIgnoreAcceleration);
  EXPECT_FALSE(machine.getModeRequest().has_value());

  machine.tick(makeInput(true, "", 0.2));
  EXPECT_FALSE(machine.getModeRequest().has_value());

  machine.tick(makeInput(true, "", 0.3));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::OFFBOARD);

  machine.tick(makeInput(true, "OFFBOARD", 0.4, true));
  EXPECT_EQ(machine.getCurrentState(), State::ReturnActive);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
  EXPECT_FALSE(machine.getSetpointTarget().has_value());
  EXPECT_FALSE(machine.getModeRequest().has_value());
}

TEST(StateMachineTest, EnterRejoinClearsFixedHoldSetpoint) {
  StateMachineConfig config;
  config.pre_offboard_setpoint_count = 0;

  BacktrackStateMachine machine(config);

  machine.tick(makeInput(true, "", 0.0));
  machine.startRecording();
  machine.lockPosition(Vec3(3.0, 4.0, 5.0), 1.2);
  machine.triggerReturn(Event::Ch7LongPress);

  machine.tick(makeInput(true, "OFFBOARD", 0.1, true));
  ASSERT_EQ(machine.getCurrentState(), State::ReturnActive);
  ASSERT_FALSE(machine.getSetpointTarget().has_value());

  machine.enterRejoin();
  machine.tick(makeInput(true, "OFFBOARD", 0.2, true));

  EXPECT_EQ(machine.getCurrentState(), State::ReturnRejoin);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
  EXPECT_FALSE(machine.getSetpointTarget().has_value());
  EXPECT_FALSE(machine.getModeRequest().has_value());
}

TEST(StateMachineTest, TrajectoryCompletionImmediatelyPublishesLandingHoldSetpoint) {
  StateMachineConfig config;
  config.pre_offboard_setpoint_count = 0;

  BacktrackStateMachine machine(config);
  machine.tick(makeInput(true, "", 0.0));
  machine.startRecording();
  machine.lockPosition(Vec3(3.0, 4.0, 5.0), 1.2);
  machine.triggerReturn(Event::Ch7LongPress);
  machine.tick(makeInput(true, "OFFBOARD", 0.1, true));
  ASSERT_EQ(machine.getCurrentState(), State::ReturnActive);

  auto finished_input = makeInput(true, "OFFBOARD", 0.2, true, true);
  machine.tick(finished_input);

  EXPECT_EQ(machine.getCurrentState(), State::LandingHover);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
  ASSERT_TRUE(machine.getSetpointTarget().has_value());
  EXPECT_TRUE(machine.getSetpointTarget()->position.isApprox(finished_input.current_position));
  EXPECT_DOUBLE_EQ(machine.getSetpointTarget()->yaw, finished_input.current_yaw);
  EXPECT_EQ(machine.getSetpointTarget()->type_mask,
            position_target_mask::kIgnoreAcceleration);
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

  EXPECT_EQ(machine.getCurrentState(), State::AutoLandYield);
  EXPECT_FALSE(machine.shouldPublishSetpoint());
  EXPECT_FALSE(machine.getModeRequest().has_value());
}

TEST(StateMachineTest, AutoLandDisarmReturnsToIdleDisarmed) {
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
  ASSERT_EQ(machine.getCurrentState(), State::AutoLandYield);

  machine.tick(makeInput(false, "AUTO.LAND", 0.3));

  EXPECT_EQ(machine.getCurrentState(), State::IdleDisarmed);
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

TEST(StateMachineTest, LandingTransitionRetriesUntilActualModeChanges) {
  StateMachineConfig config;
  config.offboard_attempt_interval_s = 1.0;
  BacktrackStateMachine machine(config);

  machine.tick(makeInput(true, "OFFBOARD", 0.0));
  machine.startRecording();
  machine.lockPosition(Vec3(3.0, 4.0, 5.0), 1.2);
  machine.triggerLanding();

  machine.tick(makeInput(true, "OFFBOARD", 0.1));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::POSCTL);
  EXPECT_TRUE(machine.shouldPublishSetpoint());
  ASSERT_TRUE(machine.getSetpointTarget().has_value());
  EXPECT_TRUE(machine.getSetpointTarget()->position.isApprox(Vec3(3.0, 4.0, 5.0)));

  machine.tick(makeInput(true, "OFFBOARD", 0.5));
  EXPECT_FALSE(machine.getModeRequest().has_value());
  EXPECT_TRUE(machine.shouldPublishSetpoint());

  machine.tick(makeInput(true, "OFFBOARD", 1.1));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::POSCTL);

  machine.tick(makeInput(true, "POSCTL", 1.2));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::AUTO_LAND);
  EXPECT_FALSE(machine.shouldPublishSetpoint());

  machine.tick(makeInput(true, "POSCTL", 1.5));
  EXPECT_FALSE(machine.getModeRequest().has_value());

  machine.tick(makeInput(true, "POSCTL", 2.2));
  ASSERT_TRUE(machine.getModeRequest().has_value());
  EXPECT_EQ(*machine.getModeRequest(), ModeRequest::AUTO_LAND);
}

}  // namespace
}  // namespace px4_backtrack::domain
