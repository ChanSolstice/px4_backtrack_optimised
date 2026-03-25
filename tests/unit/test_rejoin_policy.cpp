#include <gtest/gtest.h>

#include "px4_backtrack/domain/rejoin_policy.hpp"

namespace px4_backtrack::domain {
namespace {

std::unique_ptr<trajectory_core::Trajectory> makeStraightTrajectory() {
  trajectory_core::TrajectoryGenerator generator;
  trajectory_core::TrajectoryConfig config;

  std::vector<trajectory_core::Waypoint> waypoints;
  waypoints.emplace_back(0.0, 0.0, 1.0, 0.0, 0.0);
  waypoints.emplace_back(2.0, 0.0, 1.0, 0.0, 1.0);
  waypoints.emplace_back(4.0, 0.0, 1.0, 0.0, 2.0);

  auto result = generator.generate(waypoints, config);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_NE(result.traj, nullptr);
  return std::move(result.traj);
}

TEST(RejoinPolicyTest, TriggerAndExitUseConfiguredDistanceThresholds) {
  RejoinConfig config;
  config.trigger_dist_m = 0.5;
  config.success_dist_m = 0.1;
  RejoinPolicy policy(config);

  EXPECT_TRUE(policy.shouldTriggerRejoin(Vec3(1.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
  EXPECT_FALSE(policy.shouldTriggerRejoin(Vec3(0.2, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

  EXPECT_TRUE(policy.shouldExitRejoin(Vec3(0.05, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
  EXPECT_FALSE(policy.shouldExitRejoin(Vec3(0.2, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
}

TEST(RejoinPolicyTest, FindsNearestTimeAndLimitsVelocityForRejoinTarget) {
  RejoinConfig config;
  config.lookahead_time_s = 0.5;
  config.v_max_m_s = 0.1;
  RejoinPolicy policy(config);

  auto traj = makeStraightTrajectory();
  ASSERT_TRUE(traj);
  ASSERT_TRUE(traj->isValid());

  const Vec3 current_pos(2.1, 0.05, 1.0);
  const double nearest_time = policy.findNearestTime(current_pos, traj.get());

  EXPECT_GE(nearest_time, 0.0);
  EXPECT_LE(nearest_time, traj->duration());

  auto target = policy.computeRejoinTarget(current_pos, traj.get(), nearest_time);
  EXPECT_GE(target.t, nearest_time);
  EXPECT_LE(target.t, traj->duration());
  EXPECT_LE(target.velocity.norm(), config.v_max_m_s + 1e-6);
  EXPECT_NEAR(target.position.z(), 1.0, 1e-6);
}

TEST(RejoinPolicyTest, InvalidTrajectoryReturnsZeroTimeAndDefaultState) {
  RejoinPolicy policy{RejoinConfig()};

  EXPECT_DOUBLE_EQ(policy.findNearestTime(Vec3(0.0, 0.0, 0.0), nullptr), 0.0);

  auto target = policy.computeRejoinTarget(Vec3(0.0, 0.0, 0.0), nullptr, 1.0);
  EXPECT_DOUBLE_EQ(target.t, 0.0);
  EXPECT_TRUE(target.position.isApprox(Vec3::Zero()));
  EXPECT_TRUE(target.velocity.isApprox(Vec3::Zero()));
}

}  // namespace
}  // namespace px4_backtrack::domain
