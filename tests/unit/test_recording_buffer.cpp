#include <gtest/gtest.h>

#include "px4_backtrack/domain/recording_buffer.hpp"

namespace px4_backtrack::domain {
namespace {

TEST(RecordingBufferTest, FirstPointStartsAtZeroTimestamp) {
  RecordingBuffer buffer(10.0);

  ASSERT_TRUE(buffer.addPoint(Vec3(1.0, 2.0, 3.0), 0.25, 12.5));
  ASSERT_EQ(buffer.size(), 1u);

  const auto& points = buffer.getPoints();
  EXPECT_TRUE(points[0].position.isApprox(Vec3(1.0, 2.0, 3.0)));
  EXPECT_DOUBLE_EQ(points[0].yaw, 0.25);
  EXPECT_DOUBLE_EQ(points[0].timestamp, 0.0);
}

TEST(RecordingBufferTest, ThrottlesHighFrequencySamples) {
  RecordingBuffer buffer(10.0);  // min_dt = 0.1s

  ASSERT_TRUE(buffer.addPoint(Vec3(0.0, 0.0, 0.0), 0.0, 0.0));
  EXPECT_FALSE(buffer.addPoint(Vec3(1.0, 0.0, 0.0), 0.0, 0.05));
  EXPECT_TRUE(buffer.addPoint(Vec3(2.0, 0.0, 0.0), 0.0, 0.10));

  ASSERT_EQ(buffer.size(), 2u);
  const auto& points = buffer.getPoints();
  EXPECT_TRUE(points[1].position.isApprox(Vec3(2.0, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(points[1].timestamp, 0.10);
}

TEST(RecordingBufferTest, RejectsNonMonotonicTimestamps) {
  RecordingBuffer buffer(5.0);  // min_dt = 0.2s

  ASSERT_TRUE(buffer.addPoint(Vec3(0.0, 0.0, 0.0), 0.0, 10.0));
  ASSERT_TRUE(buffer.addPoint(Vec3(1.0, 0.0, 0.0), 0.1, 10.3));
  EXPECT_FALSE(buffer.addPoint(Vec3(2.0, 0.0, 0.0), 0.2, 10.25));

  ASSERT_EQ(buffer.size(), 2u);
  EXPECT_NEAR(buffer.getPoints().back().timestamp, 0.3, 1e-9);
}

TEST(RecordingBufferTest, ResetClearsRecordedState) {
  RecordingBuffer buffer(10.0);

  ASSERT_TRUE(buffer.addPoint(Vec3(0.0, 0.0, 0.0), 0.0, 1.0));
  ASSERT_FALSE(buffer.empty());

  buffer.reset();

  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(buffer.size(), 0u);

  ASSERT_TRUE(buffer.addPoint(Vec3(5.0, 0.0, 0.0), 1.0, 20.0));
  ASSERT_EQ(buffer.size(), 1u);
  EXPECT_DOUBLE_EQ(buffer.getPoints()[0].timestamp, 0.0);
}

}  // namespace
}  // namespace px4_backtrack::domain
