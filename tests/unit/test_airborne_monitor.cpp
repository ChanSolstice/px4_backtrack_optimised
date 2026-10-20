#include <gtest/gtest.h>

#include <cmath>

#include "px4_backtrack/domain/airborne_monitor.hpp"

namespace px4_backtrack::domain {
namespace {

AirborneMonitorConfig makeConfig() {
  AirborneMonitorConfig config{};
  config.airborne_min_height_m = 0.5;
  config.fcu_state_stale_s = 3.0;
  return config;
}

/// 地面上（z=0.1）解锁，然后起飞到 3 m
AirborneMonitor armOnGroundAndClimb() {
  AirborneMonitor monitor(makeConfig());
  monitor.onHeight(0.1);
  monitor.onArmed(false);
  monitor.onArmed(true);
  monitor.onLandedState(0.0, LandedState::OnGround);
  monitor.update(0.0);
  monitor.onHeight(3.0);
  monitor.onLandedState(5.0, LandedState::InAir);
  monitor.update(5.0);
  return monitor;
}

TEST(AirborneMonitorTest, ArmedOnGroundIsNotAirborneUntilClimbed) {
  AirborneMonitor monitor(makeConfig());
  monitor.onHeight(0.1);
  monitor.onArmed(false);
  monitor.onArmed(true);
  monitor.onLandedState(0.0, LandedState::OnGround);
  monitor.update(0.0);
  EXPECT_EQ(monitor.status().arm_height, ArmHeightRef::Known);
  EXPECT_FALSE(monitor.status().airborne);
  EXPECT_NEAR(monitor.status().height_above_arm_m, 0.0, 1e-9);

  monitor.onHeight(3.0);
  monitor.onLandedState(5.0, LandedState::InAir);
  monitor.update(5.0);
  EXPECT_TRUE(monitor.status().airborne);
  EXPECT_EQ(monitor.status().landed_state, LandedState::InAir);
  EXPECT_NEAR(monitor.status().height_above_arm_m, 2.9, 1e-9);
}

TEST(AirborneMonitorTest, TouchdownNotReportedByFcuIsNotAirborne) {
  // S13（仿真 H06a）：触地瞬间关遥控器，摇杆回中，飞控没有报告落地
  AirborneMonitor monitor = armOnGroundAndClimb();
  ASSERT_TRUE(monitor.status().airborne);

  monitor.onHeight(0.3);
  monitor.onLandedState(20.0, LandedState::InAir);
  monitor.update(20.0);
  EXPECT_EQ(monitor.status().landed_state, LandedState::InAir);
  EXPECT_NEAR(monitor.status().height_above_arm_m, 0.2, 1e-9);
  EXPECT_FALSE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, FcuOnGroundOverridesHeight) {
  // 降落在比起飞点高的平台上：高度条件满足，以飞控的落地判定为准
  AirborneMonitor monitor = armOnGroundAndClimb();
  monitor.onLandedState(20.0, LandedState::OnGround);
  monitor.update(20.0);
  EXPECT_FALSE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, HeightMustExceedThreshold) {
  AirborneMonitor monitor = armOnGroundAndClimb();
  monitor.onHeight(0.55);  // 高出 0.45 m
  monitor.update(6.0);
  EXPECT_FALSE(monitor.status().airborne);
  monitor.onHeight(0.65);  // 高出 0.55 m
  monitor.update(6.1);
  EXPECT_TRUE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, MissingOrStaleLandedStateCountsAsNotOnGround) {
  AirborneMonitor monitor(makeConfig());
  monitor.onHeight(0.0);
  monitor.onArmed(false);
  monitor.onArmed(true);
  monitor.onHeight(2.0);

  // 没有收到 extended_state：只看高度
  monitor.update(1.0);
  EXPECT_EQ(monitor.status().landed_state, LandedState::Undefined);
  EXPECT_TRUE(monitor.status().airborne);

  // R-58：ON_GROUND 新鲜时生效，超过 fcu_state_stale_s 未更新视为未定义
  monitor.onLandedState(2.0, LandedState::OnGround);
  monitor.update(4.9);
  EXPECT_EQ(monitor.status().landed_state, LandedState::OnGround);
  EXPECT_FALSE(monitor.status().airborne);
  monitor.update(5.1);
  EXPECT_EQ(monitor.status().landed_state, LandedState::Undefined);
  EXPECT_TRUE(monitor.status().airborne);

  // 状态未定义同样视为未报告落地
  monitor.onLandedState(6.0, LandedState::Undefined);
  monitor.update(6.0);
  EXPECT_TRUE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, NodeStartedWhileArmedUsesLandedStateOnly) {
  // R-58：空中重启，第一次收到的飞控状态就是已解锁，没有解锁高度
  AirborneMonitor monitor(makeConfig());
  monitor.onHeight(10.0);
  monitor.onArmed(true);
  monitor.onLandedState(0.0, LandedState::InAir);
  monitor.update(0.0);
  EXPECT_EQ(monitor.status().arm_height, ArmHeightRef::NotApplicable);
  EXPECT_TRUE(std::isnan(monitor.status().height_above_arm_m));
  EXPECT_TRUE(monitor.status().airborne);

  monitor.onLandedState(1.0, LandedState::OnGround);
  monitor.update(1.0);
  EXPECT_FALSE(monitor.status().airborne);

  // 上锁后再解锁，回到正常的解锁高度判定
  monitor.onArmed(false);
  monitor.onHeight(0.0);
  monitor.onArmed(true);
  monitor.onLandedState(2.0, LandedState::InAir);
  monitor.update(2.0);
  EXPECT_EQ(monitor.status().arm_height, ArmHeightRef::Known);
  EXPECT_FALSE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, ArmBeforeOdometryUsesLandedStateOnly) {
  // 解锁时还没有里程计：与空中重启一样只按飞控落地状态判定
  AirborneMonitor monitor(makeConfig());
  monitor.onArmed(false);
  monitor.onArmed(true);
  monitor.onHeight(0.2);
  monitor.update(0.0);
  EXPECT_EQ(monitor.status().arm_height, ArmHeightRef::NotApplicable);
  EXPECT_TRUE(monitor.status().airborne);

  monitor.onLandedState(0.1, LandedState::OnGround);
  monitor.update(0.1);
  EXPECT_FALSE(monitor.status().airborne);
}

TEST(AirborneMonitorTest, EachFlightCycleRecordsItsOwnArmHeight) {
  AirborneMonitor monitor = armOnGroundAndClimb();

  // 上锁后被搬到 2 m 高的平台上再解锁
  monitor.onArmed(false);
  monitor.update(30.0);
  EXPECT_EQ(monitor.status().arm_height, ArmHeightRef::None);
  EXPECT_FALSE(monitor.status().airborne);
  monitor.onHeight(2.0);
  monitor.onArmed(true);
  monitor.onHeight(2.3);
  monitor.onLandedState(31.0, LandedState::InAir);
  monitor.update(31.0);
  EXPECT_NEAR(monitor.status().height_above_arm_m, 0.3, 1e-9);
  EXPECT_FALSE(monitor.status().airborne);

  // 重复的已解锁状态不会重新记录解锁高度
  monitor.onHeight(3.0);
  monitor.onArmed(true);
  monitor.update(32.0);
  EXPECT_NEAR(monitor.status().height_above_arm_m, 1.0, 1e-9);
  EXPECT_TRUE(monitor.status().airborne);
}

}  // namespace
}  // namespace px4_backtrack::domain
