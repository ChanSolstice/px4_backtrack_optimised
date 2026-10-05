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


}  // namespace
}  // namespace px4_backtrack::domain
