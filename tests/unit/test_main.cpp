#include <gtest/gtest.h>
#include <ros/time.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
