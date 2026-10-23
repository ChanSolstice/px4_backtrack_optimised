#include <gtest/gtest.h>

#include <vector>

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

/// 沿 x 轴的录制点，时间戳间隔 0.1 s；录制频率上限取 100 Hz，不触发节流
RecordingBuffer lineAlongX(const std::vector<double>& xs) {
  RecordingBuffer buffer(100.0);
  for (size_t i = 0; i < xs.size(); ++i) {
    buffer.addPoint(Vec3(xs[i], 0.0, 1.0), 0.0, 0.1 * static_cast<double>(i));
  }
  return buffer;
}

TEST(RecordingBufferTest, TruncateKeepsPrefix) {
  auto buffer = lineAlongX({0.0, 1.0, 2.0, 3.0, 4.0});
  buffer.truncate(10);
  EXPECT_EQ(buffer.size(), 5u);
  buffer.truncate(2);
  ASSERT_EQ(buffer.size(), 3u);
  EXPECT_DOUBLE_EQ(buffer.getPoints().back().position.x(), 2.0);
  buffer.truncate(0);
  ASSERT_EQ(buffer.size(), 1u);
  EXPECT_DOUBLE_EQ(buffer.getPoints().back().position.x(), 0.0);
}

TEST(RecordingBufferTest, ResumeContinuesTimestampAfterLastPoint) {
  // 第 7.4 节：截断并暂停录制后，第一个新点接在末点之后 1/max_hz
  RecordingBuffer buffer(10.0);
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(buffer.addPoint(Vec3(static_cast<double>(i), 0.0, 1.0), 0.0, 0.25 * i));
  }
  buffer.truncate(1);  // 末点时间戳 0.25
  buffer.resume();
  ASSERT_TRUE(buffer.addPoint(Vec3(5.0, 0.0, 1.0), 0.0, 50.0));
  EXPECT_NEAR(buffer.getPoints().back().timestamp, 0.35, 1e-9);
  EXPECT_FALSE(buffer.addPoint(Vec3(5.1, 0.0, 1.0), 0.0, 50.05));  // 节流照常
  ASSERT_TRUE(buffer.addPoint(Vec3(5.2, 0.0, 1.0), 0.0, 50.15));
  EXPECT_NEAR(buffer.getPoints().back().timestamp, 0.5, 1e-9);
  EXPECT_EQ(buffer.size(), 4u);

  // 空缓存上的 resume 不影响之后的第一个点
  RecordingBuffer empty(10.0);
  empty.resume();
  ASSERT_TRUE(empty.addPoint(Vec3::Zero(), 0.0, 7.0));
  ASSERT_TRUE(empty.addPoint(Vec3::Zero(), 0.0, 7.5));
  EXPECT_NEAR(empty.getPoints().back().timestamp, 0.5, 1e-9);
}

TEST(RecordingBufferTest, NearestBeforeSearchesWindowBackwardOnly) {
  // 每 0.5 m 一个点，x = 0～5，索引 0～10
  std::vector<double> xs;
  for (int i = 0; i <= 10; ++i) xs.push_back(0.5 * i);
  const auto buffer = lineAlongX(xs);

  // 窗口 2 m：只看索引 10～6（x = 5～3），离 x = 3.2 最近的是 x = 3.0
  EXPECT_EQ(buffer.nearestBefore(10, Vec3(3.2, 0.0, 1.0), 2.0), 6u);
  // 参考点远在窗口之外：最多退到窗口边界
  EXPECT_EQ(buffer.nearestBefore(10, Vec3(0.0, 0.0, 1.0), 2.0), 6u);
  // 只减不增：参考点在游标之后时游标不动
  EXPECT_EQ(buffer.nearestBefore(6, Vec3(4.8, 0.0, 1.0), 2.0), 6u);
  // 距离相同时取更小的索引
  EXPECT_EQ(buffer.nearestBefore(10, Vec3(3.75, 0.0, 1.0), 2.0), 7u);
}

TEST(RecordingBufferTest, NearestBeforeCrossesHoverClusterInOneStep) {
  // 悬停堆积的密集点不占弧长，一次搜索就能跨过
  std::vector<double> xs = {0.0, 0.5, 1.0, 1.5};
  for (int i = 0; i < 20; ++i) xs.push_back(2.0);
  xs.push_back(2.5);
  xs.push_back(3.0);
  const auto buffer = lineAlongX(xs);
  const size_t last = buffer.size() - 1;
  EXPECT_EQ(buffer.nearestBefore(last, Vec3(1.5, 0.0, 1.0), 2.0), 3u);
}

TEST(RecordingBufferTest, NearestBeforeStaysOnLocalBranchOfSelfCrossingTrail) {
  // 先向东 0～4 m，再向北 2 m、向西 2 m、向南穿过起始段 (2, 0) 附近
  RecordingBuffer buffer(100.0);
  std::vector<Vec3> trail;
  for (int i = 0; i <= 8; ++i) trail.emplace_back(0.5 * i, 0.0, 1.0);        // 0～8
  for (int i = 1; i <= 4; ++i) trail.emplace_back(4.0, 0.5 * i, 1.0);        // 9～12
  for (int i = 1; i <= 4; ++i) trail.emplace_back(4.0 - 0.5 * i, 2.0, 1.0);  // 13～16
  for (int i = 1; i <= 5; ++i) trail.emplace_back(2.0, 2.0 - 0.5 * i, 1.0);  // 17～21
  for (size_t i = 0; i < trail.size(); ++i) {
    buffer.addPoint(trail[i], 0.0, 0.1 * static_cast<double>(i));
  }
  // 返航从末点 (2, -0.5) 出发，路径投影点在 (2, 0.1)：起始段的 (2, 0)（索引 4）不在窗口内
  const size_t cursor = buffer.nearestBefore(buffer.size() - 1, Vec3(2.0, 0.1, 1.0), 2.0);
  EXPECT_EQ(cursor, 20u);  // (2, 0)，后一段上的点
}

TEST(RecordingBufferTest, ContinuousFromFindsLastGap) {
  // D30：从末点往前，相邻两点超过 max_gap 的位置是断点
  EXPECT_EQ(lineAlongX({0.0, 0.5, 1.0, 1.5}).continuousFrom(1.0), 0u);
  EXPECT_EQ(lineAlongX({0.0, 0.5, 4.0, 4.5, 5.0}).continuousFrom(1.0), 2u);
  EXPECT_EQ(lineAlongX({0.0, 3.0, 3.5, 8.0}).continuousFrom(1.0), 3u);  // 断点就在末点前
  EXPECT_EQ(lineAlongX({0.0, 1.0, 2.0}).continuousFrom(1.0), 0u);       // 正好等于不算断点
  EXPECT_EQ(RecordingBuffer(10.0).continuousFrom(1.0), 0u);
}

TEST(RecordingBufferTest, FirstWithinReturnsEarliestPointInRange) {
  // D33：[0, end) 中离参考点不超过半径的点里取最早录制的；往返时去程上的点更早
  const RecordingBuffer buffer = lineAlongX({0.0, 0.5, 1.0, 1.5, 2.0, 1.5, 1.0});
  // 最近的是索引 3、5（0.1 m），范围内最早的是索引 2
  EXPECT_EQ(buffer.firstWithin(7, Vec3(1.5, 0.1, 1.0), 0.7).value_or(99), 2u);
  EXPECT_EQ(buffer.firstWithin(7, Vec3(1.0, 0.6, 1.0), 0.6).value_or(99), 2u);  // 等于半径也算
  EXPECT_FALSE(buffer.firstWithin(7, Vec3(1.0, 0.6, 1.0), 0.5));
  EXPECT_FALSE(buffer.firstWithin(2, Vec3(1.0, 0.6, 1.0), 0.7));  // 不含 end
  EXPECT_FALSE(buffer.firstWithin(0, Vec3(0.0, 0.0, 1.0), 0.7));
}

}  // namespace
}  // namespace px4_backtrack::domain
