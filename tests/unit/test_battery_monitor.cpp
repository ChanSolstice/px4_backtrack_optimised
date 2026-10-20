#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "px4_backtrack/domain/battery_monitor.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kDt = 0.05;  // tick 20Hz

BatteryMonitorConfig makeConfig() {
  BatteryMonitorConfig config{};
  config.enable_low_battery = true;
  config.low_battery_threshold = 0.15;
  config.low_battery_confirm_s = 3.0;
  return config;
}

/// 电量读数 1Hz、tick 20Hz：在 [from, to) 内每秒整点给一次读数，每拍 update
void feed(BatteryMonitor& monitor, double from, double to, double percentage) {
  const int k_from = static_cast<int>(std::lround(from / kDt));
  const int k_to = static_cast<int>(std::lround(to / kDt));
  for (int k = k_from; k < k_to; ++k) {
    const double t = k * kDt;
    if (k % 20 == 0) {
      monitor.onReading(t, percentage);
    }
    monitor.update(t);
  }
}

TEST(BatteryMonitorTest, ConfirmsAfterContinuousLowAndLatches) {
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 3.0, 0.12);  // 最后一拍 t=2.95
  EXPECT_FALSE(monitor.status().low);
  feed(monitor, 3.0, 3.05, 0.12);
  EXPECT_TRUE(monitor.status().low);
  EXPECT_DOUBLE_EQ(monitor.status().percentage, 0.12);

  // 电量回升（负载下降、电压回弹）不取消锁存
  feed(monitor, 3.05, 10.0, 0.30);
  EXPECT_TRUE(monitor.status().low);
}

TEST(BatteryMonitorTest, ValidReadingAboveThresholdRestartsConfirmation) {
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 2.0, 0.12);
  feed(monitor, 2.0, 3.0, 0.16);
  feed(monitor, 3.0, 5.95, 0.12);  // 从 t=3.0 重新计时
  EXPECT_FALSE(monitor.status().low);
  feed(monitor, 5.95, 6.05, 0.12);
  EXPECT_TRUE(monitor.status().low);
}

TEST(BatteryMonitorTest, UnknownBatteryIsInvalidNotLow) {
  // S14：飞控上报电量未知时 MAVROS 换算出 -0.01
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 30.0, -0.01);
  EXPECT_FALSE(monitor.status().low);
  EXPECT_TRUE(monitor.status().invalid);
  EXPECT_TRUE(std::isnan(monitor.status().percentage));
}

TEST(BatteryMonitorTest, ValidityCheck) {
  EXPECT_TRUE(BatteryMonitor::isValidReading(0.0));
  EXPECT_TRUE(BatteryMonitor::isValidReading(0.5));
  EXPECT_TRUE(BatteryMonitor::isValidReading(1.0));
  EXPECT_FALSE(BatteryMonitor::isValidReading(-0.01));
  EXPECT_FALSE(BatteryMonitor::isValidReading(1.01));
  EXPECT_FALSE(BatteryMonitor::isValidReading(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(BatteryMonitor::isValidReading(std::numeric_limits<double>::infinity()));

  BatteryMonitor monitor(makeConfig());
  monitor.onReading(0.0, 0.8);
  EXPECT_FALSE(monitor.status().invalid);
  monitor.onReading(1.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(monitor.status().invalid);
  // 无效读数不覆盖最近一次有效读数
  EXPECT_DOUBLE_EQ(monitor.status().percentage, 0.8);
  monitor.onReading(2.0, 0.7);
  EXPECT_FALSE(monitor.status().invalid);
}

TEST(BatteryMonitorTest, NoReadingYetIsInvalid) {
  // R-59：还没有收到读数时告警，起飞前检查能发现电量话题缺失
  BatteryMonitor monitor(makeConfig());
  EXPECT_TRUE(monitor.status().invalid);
  monitor.update(10.0);
  EXPECT_TRUE(monitor.status().invalid);
  EXPECT_FALSE(monitor.status().low);
}

TEST(BatteryMonitorTest, InvalidReadingsNeitherResetNorAdvanceConfirmation) {
  // R-60
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 1.0, 0.12);
  feed(monitor, 1.0, 5.0, -0.01);  // 距第一次低读数已超过 3 s，但当前读数无效
  EXPECT_FALSE(monitor.status().low);
  EXPECT_TRUE(monitor.status().invalid);

  // 恢复有效低读数时，确认计时从 t=0 算起，立即置位
  monitor.onReading(5.0, 0.12);
  EXPECT_TRUE(monitor.status().low);
}

TEST(BatteryMonitorTest, InvalidReadingKeepsLatchedLow) {
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 4.0, 0.12);
  ASSERT_TRUE(monitor.status().low);
  feed(monitor, 4.0, 10.0, -0.01);
  EXPECT_TRUE(monitor.status().low);
  EXPECT_TRUE(monitor.status().invalid);
}

TEST(BatteryMonitorTest, ResetStartsNewFlightCycle) {
  BatteryMonitor monitor(makeConfig());
  feed(monitor, 0.0, 4.0, 0.12);
  ASSERT_TRUE(monitor.status().low);

  // 新的飞行周期：清除锁存；电池没换时重新确认
  monitor.reset();
  EXPECT_FALSE(monitor.status().low);
  feed(monitor, 4.05, 7.05, 0.12);  // 复位后第一拍 t=4.05 起算
  EXPECT_FALSE(monitor.status().low);
  feed(monitor, 7.05, 7.1, 0.12);
  EXPECT_TRUE(monitor.status().low);
}

TEST(BatteryMonitorTest, DisabledNeverLowOrInvalid) {
  BatteryMonitorConfig config = makeConfig();
  config.enable_low_battery = false;
  BatteryMonitor monitor(config);
  EXPECT_FALSE(monitor.status().invalid);
  feed(monitor, 0.0, 10.0, 0.05);
  feed(monitor, 10.0, 12.0, -0.01);
  EXPECT_FALSE(monitor.status().low);
  EXPECT_FALSE(monitor.status().invalid);
}

}  // namespace
}  // namespace px4_backtrack::domain
