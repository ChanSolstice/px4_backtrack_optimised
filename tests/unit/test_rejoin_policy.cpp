#include <gtest/gtest.h>

#include "px4_backtrack/domain/rejoin_policy.hpp"

namespace px4_backtrack::domain {
namespace {

TEST(RejoinPolicyTest, TriggerAndExitRequireDistanceAndVelocityConvergence) {
  RejoinConfig config;
  config.trigger_dist_m = 0.5;
  config.success_dist_m = 0.1;
  RejoinPolicy policy(config);

  EXPECT_TRUE(policy.shouldTriggerRejoin(1.0));
  EXPECT_FALSE(policy.shouldTriggerRejoin(0.2));

  const Vec3 path_velocity(0.6, 0.0, 0.0);
  EXPECT_TRUE(policy.shouldExitRejoin(
      0.05, Vec3(0.4, 0.0, 0.0), path_velocity, path_velocity));
  EXPECT_FALSE(policy.shouldExitRejoin(
      0.2, path_velocity, path_velocity, path_velocity));
  EXPECT_FALSE(policy.shouldExitRejoin(
      0.05, Vec3::Zero(), path_velocity, path_velocity));
  EXPECT_FALSE(policy.shouldExitRejoin(
      0.05, path_velocity, path_velocity, Vec3::Zero()));
}

TEST(RejoinPolicyTest, FarDeviationLimitsSpeedWithoutChangingPathPose) {
  RejoinConfig config;
  config.trigger_dist_m = 0.5;
  config.success_dist_m = 0.1;
  config.v_max_m_s = 0.3;
  RejoinPolicy policy(config);

  trajectory_core::TrajectoryState path_target;
  path_target.position = Vec3(1.0, 2.0, 1.0);
  path_target.velocity = Vec3(1.0, 0.0, 0.0);
  path_target.yaw = 1.2;
  path_target.yaw_rate = 0.2;

  auto limited = policy.applyRejoinLimits(
      path_target, 0.8, path_target.velocity);
  EXPECT_TRUE(limited.position.isApprox(path_target.position));
  EXPECT_DOUBLE_EQ(limited.yaw, path_target.yaw);
  EXPECT_DOUBLE_EQ(limited.yaw_rate, path_target.yaw_rate);
  EXPECT_LE((limited.velocity - path_target.velocity).norm(), 0.3 + 1e-9);
  EXPECT_LT(limited.velocity.norm(), path_target.velocity.norm());
  EXPECT_FALSE(limited.acceleration.has_value());

  auto converged = limited;
  for (int i = 0; i < 4; ++i) {
    converged = policy.applyRejoinLimits(
        path_target, 0.8, converged.velocity);
  }
  EXPECT_NEAR(converged.velocity.norm(), config.v_max_m_s, 1e-9);
}

TEST(RejoinPolicyTest, SpeedRecoversSmoothlyAsCrossTrackErrorConverges) {
  RejoinConfig config;
  config.trigger_dist_m = 0.5;
  config.success_dist_m = 0.1;
  config.v_max_m_s = 0.3;
  RejoinPolicy policy(config);

  trajectory_core::TrajectoryState path_target;
  path_target.velocity = Vec3(1.0, 0.0, 0.0);

  const auto far = policy.applyRejoinLimits(
      path_target, 0.5, Vec3(0.3, 0.0, 0.0));
  const auto middle = policy.applyRejoinLimits(
      path_target, 0.3, far.velocity);
  const auto near = policy.applyRejoinLimits(
      path_target, 0.05, Vec3(0.8, 0.0, 0.0));

  EXPECT_NEAR(far.velocity.norm(), 0.3, 1e-9);
  EXPECT_GT(middle.velocity.norm(), far.velocity.norm());
  EXPECT_LT(middle.velocity.norm(), path_target.velocity.norm());
  EXPECT_NEAR(near.velocity.norm(), path_target.velocity.norm(), 1e-9);
}

}  // namespace
}  // namespace px4_backtrack::domain
