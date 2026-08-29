#include <cmath>

#include <gtest/gtest.h>

#include "px4_backtrack/adapters/ros_subscribers.hpp"

namespace px4_backtrack::adapters {
namespace {

nav_msgs::Odometry makeOdometry(double yaw, const domain::Vec3& velocity_child) {
  nav_msgs::Odometry msg;
  msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
  msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
  msg.twist.twist.linear.x = velocity_child.x();
  msg.twist.twist.linear.y = velocity_child.y();
  msg.twist.twist.linear.z = velocity_child.z();
  return msg;
}

TEST(RosSubscribersTest, RotatesChildVelocityIntoParentEnuFrame) {
  const auto msg = makeOdometry(M_PI, domain::Vec3(0.8, -0.2, 0.1));

  const domain::Vec3 velocity_parent = velocityInParentFrame(msg);

  EXPECT_TRUE(velocity_parent.isApprox(domain::Vec3(-0.8, 0.2, 0.1), 1e-12));
}

TEST(RosSubscribersTest, PreservesVelocityAtZeroYaw) {
  const auto msg = makeOdometry(0.0, domain::Vec3(0.4, -0.3, 0.2));

  EXPECT_TRUE(velocityInParentFrame(msg).isApprox(
      domain::Vec3(0.4, -0.3, 0.2), 1e-12));
}

}  // namespace
}  // namespace px4_backtrack::adapters
