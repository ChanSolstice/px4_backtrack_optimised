#include <gtest/gtest.h>

#include "px4_backtrack/domain/link_monitor.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kDt = 0.05;  // RC帧与tick都是20Hz
constexpr uint16_t kNormal = LinkMonitor::kCh18NormalValue;
constexpr uint16_t kFailsafe = LinkMonitor::kCh18FailsafeValue;

LinkMonitorConfig makeConfig() {
  LinkMonitorConfig config{};
  config.enable_ch18 = true;
  config.ch18_failsafe_threshold = 900;
  config.ch18_confirm_s = 1.0;
  config.link_loss_leak_rate = 0.2;
  config.enable_rc_loss = true;
  config.rc_loss_timeout_s = 0.15;
  config.rc_loss_debounce_s = 0.1;
  config.link_recover_s = 2.0;
  config.link_degraded_window_s = 5.0;
  return config;
}

std::vector<uint16_t> makeFrame(uint16_t ch18, std::size_t size = 18) {
  std::vector<uint16_t> channels(size, 1500);
  if (size > 17) {
    channels[17] = ch18;
  }
  return channels;
}

/// 按20Hz驱动：每一步先收一帧，再tick一次，时间取 k*kDt 避免累加误差
struct Rig {
  explicit Rig(const LinkMonitorConfig& config = makeConfig()) : link(config) {}

  double now() const { return k * kDt; }

  void frame(uint16_t ch18, std::size_t size = 18) {
    const double t = now();
    link.onRcFrame(t, makeFrame(ch18, size));
    link.update(t);
    ++k;
  }

  void frames(int n, uint16_t ch18) {
    for (int i = 0; i < n; ++i) {
      frame(ch18);
    }
  }

  /// 没有RC帧，只有tick
  void tick() {
    link.update(now());
    ++k;
  }

  const LinkStatus& status() const { return link.status(); }

  LinkMonitor link;
  int k{0};
};

TEST(LinkMonitorTest, ContinuousLossConfirmsAfterOneSecond) {
  Rig rig;
  rig.frames(20, kNormal);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);

  // 第一帧失控帧在 t=1.0，持续 0.95 s 时尚未确认
  rig.frames(20, kFailsafe);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_TRUE(rig.status().degraded);
  EXPECT_TRUE(rig.status().ch18_low_now);
  EXPECT_NEAR(rig.status().loss_bucket_s, 0.95, 1e-9);

  // 持续 1.0 s 时确认，与 dev 的连续判定一致
  rig.frame(kFailsafe);
  EXPECT_TRUE(rig.status().lost);
  EXPECT_EQ(rig.status().source, LinkLossSource::Ch18);
}

TEST(LinkMonitorTest, IntermittentLossConfirmsOnSixthDropout) {
  // S20：每秒掉线 0.3 s（6 帧失控 + 14 帧正常）
  Rig rig;
  double first_lost_t = -1.0;
  for (int cycle = 0; cycle < 10 && first_lost_t < 0.0; ++cycle) {
    for (int i = 0; i < 20; ++i) {
      const double t = rig.now();
      rig.frame(i < 6 ? kFailsafe : kNormal);
      if (rig.status().lost) {
        first_lost_t = t;
        break;
      }
      // 确认之前一直只报退化
      EXPECT_TRUE(rig.status().degraded) << "t=" << t;
    }
  }
  // 每秒净增 0.3 - 0.7*0.2 = 0.16，第 6 次掉线开始时漏桶为 0.8，再掉 0.2 s 确认
  EXPECT_NEAR(first_lost_t, 5.2, 1e-6);
  EXPECT_EQ(rig.status().source, LinkLossSource::Ch18);
}

TEST(LinkMonitorTest, SparseDropoutsOnlyDegrade) {
  // 每 2 s 掉线 0.1 s：漏桶最多 0.1，不会确认，退化一直成立
  Rig rig;
  for (int cycle = 0; cycle < 30; ++cycle) {
    rig.frames(2, kFailsafe);
    rig.frames(38, kNormal);
    EXPECT_FALSE(rig.status().lost);
    EXPECT_TRUE(rig.status().degraded);
    EXPECT_LE(rig.status().loss_bucket_s, 0.1 + 1e-9);
  }

  // 最后一次掉线结束于 t=58.1，此后正常超过 5 s，退化清除
  rig.frames(61, kNormal);  // 到 t=63.0
  EXPECT_TRUE(rig.status().degraded);
  rig.frames(4, kNormal);   // 到 t=63.2
  EXPECT_FALSE(rig.status().degraded);
}

TEST(LinkMonitorTest, RecoveryNeedsTwoSecondsOfNormalFramesAndKeepsBucket) {
  Rig rig;
  rig.frames(30, kFailsafe);  // 1.5 s，已确认
  ASSERT_TRUE(rig.status().lost);
  EXPECT_DOUBLE_EQ(rig.status().loss_bucket_s, 1.0);

  // 恢复 1.0 s 后掉一帧（t=2.5），连续正常从下一帧（t=2.55）重新计时
  rig.frames(20, kNormal);
  EXPECT_TRUE(rig.status().lost);
  rig.frame(kFailsafe);
  rig.frames(40, kNormal);  // 到 t=4.5，连续正常 1.95 s
  EXPECT_TRUE(rig.status().lost);
  rig.frame(kNormal);       // t=4.55，连续正常 2.0 s
  EXPECT_FALSE(rig.status().lost);
  EXPECT_EQ(rig.status().source, LinkLossSource::None);
  // 恢复后仍处于退化窗口
  EXPECT_TRUE(rig.status().degraded);

  // 漏桶不清零（此时 0.45）：再次连续掉线时更快确认。
  // 第一帧失控帧时漏桶 0.44，之后再 12 帧确认（共 13 帧）；从零开始共需 21 帧
  EXPECT_NEAR(rig.status().loss_bucket_s, 0.45, 1e-6);
  int failsafe_frames = 0;
  while (!rig.status().lost && failsafe_frames < 100) {
    rig.frame(kFailsafe);
    ++failsafe_frames;
  }
  EXPECT_EQ(failsafe_frames, 13);
}

TEST(LinkMonitorTest, BucketIsCappedSoLongLossDoesNotLinger) {
  // R-55：失控 5 分钟后漏桶只有 ch18_confirm_s
  Rig rig;
  rig.frames(6000, kFailsafe);
  ASSERT_TRUE(rig.status().lost);
  EXPECT_DOUBLE_EQ(rig.status().loss_bucket_s, 1.0);

  // 第一帧正常帧在 t=300.0，连续正常 2 s 时恢复，漏桶为 0.6
  rig.frames(40, kNormal);
  EXPECT_TRUE(rig.status().lost);
  rig.frame(kNormal);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_NEAR(rig.status().loss_bucket_s, 0.6, 1e-6);

  // 再正常 3 s 后漏桶归零，单帧失控帧不会确认
  rig.frames(60, kNormal);
  EXPECT_NEAR(rig.status().loss_bucket_s, 0.0, 1e-9);
  rig.frame(kFailsafe);
  rig.frame(kNormal);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_TRUE(rig.status().degraded);
}

TEST(LinkMonitorTest, RcTimeoutConfirmsAfterTimeoutPlusDebounceAndRecovers) {
  Rig rig;
  rig.frames(21, kNormal);  // 最后一帧在 t=1.0

  // 0.15 s 起算超时并报退化，0.25 s 确认
  rig.tick();  // 1.05
  rig.tick();  // 1.10
  EXPECT_FALSE(rig.status().rc_timeout);
  EXPECT_FALSE(rig.status().degraded);
  rig.tick();  // 1.15
  EXPECT_TRUE(rig.status().rc_timeout);
  EXPECT_TRUE(rig.status().degraded);
  EXPECT_FALSE(rig.status().lost);
  rig.tick();  // 1.20
  EXPECT_FALSE(rig.status().lost);
  rig.tick();  // 1.25
  EXPECT_TRUE(rig.status().lost);
  EXPECT_EQ(rig.status().source, LinkLossSource::RcTimeout);

  // t=2.0 起恢复收帧：从这一帧起连续正常 2 s 才清除
  while (rig.now() < 2.0 - 1e-9) {
    rig.tick();
  }
  rig.frames(40, kNormal);  // 2.0 .. 3.95
  EXPECT_FALSE(rig.status().rc_timeout);
  EXPECT_TRUE(rig.status().lost);
  rig.frame(kNormal);       // 4.0
  EXPECT_FALSE(rig.status().lost);
  EXPECT_TRUE(rig.status().degraded);
}

TEST(LinkMonitorTest, SelfStallIsNotRcTimeout) {
  // R-61：本模块卡顿 1.3 s，卡顿后的第一拍先于排队的RC帧执行
  Rig rig;
  rig.frames(21, kNormal);  // 最后一帧在 t=1.0
  rig.k = 46;               // 下一拍在 t=2.3
  rig.tick();
  EXPECT_TRUE(rig.status().rc_timeout);  // 原始事实：距最近一帧已超时
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);

  // 排队的帧随后到达，链路一直正常
  rig.frames(10, kNormal);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);
}

TEST(LinkMonitorTest, StallFollowedByRealSilenceConfirmsOneTickLater) {
  Rig rig;
  rig.frames(21, kNormal);  // 最后一帧在 t=1.0
  rig.k = 46;
  rig.tick();               // 2.3：卡顿后的第一拍不判
  EXPECT_FALSE(rig.status().lost);
  rig.tick();               // 2.35：仍然没有帧，确认
  EXPECT_TRUE(rig.status().lost);
  EXPECT_EQ(rig.status().source, LinkLossSource::RcTimeout);
}

TEST(LinkMonitorTest, RecoveryRequiresFreshFrames) {
  // 恢复期间本模块卡顿：卡顿后的第一拍没有新帧，不能据此判定恢复
  Rig rig;
  rig.frames(30, kFailsafe);  // t=0..1.45，已确认
  ASSERT_TRUE(rig.status().lost);
  rig.frames(30, kNormal);    // t=1.5..2.95，连续正常 1.45 s
  rig.k = 80;                 // 下一拍在 t=4.0，距最后一次异常（t=1.5）已 2.5 s
  rig.tick();
  EXPECT_TRUE(rig.status().lost);

  // 卡顿期间排队的帧是失控帧
  rig.frames(5, kFailsafe);
  EXPECT_TRUE(rig.status().lost);
}

TEST(LinkMonitorTest, UnexpectedValuesAreFlaggedButThresholdStillApplies) {
  // S22：原生固件的数字通道值 998、1998 不低于阈值，失控检测失效但会告警
  Rig rig;
  rig.frames(40, 998);
  EXPECT_TRUE(rig.status().ch18_unexpected);
  EXPECT_EQ(rig.status().ch18_value, 998);
  EXPECT_FALSE(rig.status().ch18_low_now);
  EXPECT_FALSE(rig.status().lost);

  rig.frame(1998);
  EXPECT_TRUE(rig.status().ch18_unexpected);
  EXPECT_FALSE(rig.status().ch18_low_now);

  rig.frame(kNormal);
  EXPECT_FALSE(rig.status().ch18_unexpected);

  rig.frame(kFailsafe);
  EXPECT_FALSE(rig.status().ch18_unexpected);
  EXPECT_TRUE(rig.status().ch18_low_now);

  // 其他低值：告警，同时按阈值算失控帧
  rig.frame(850);
  EXPECT_TRUE(rig.status().ch18_unexpected);
  EXPECT_TRUE(rig.status().ch18_low_now);
}

TEST(LinkMonitorTest, MissingCh18IsReportedButNeverCountedAsFailsafe) {
  Rig rig;
  // 还没有收到RC消息
  EXPECT_TRUE(rig.status().ch18_unavailable);

  // 通道数不足：告警，但不算失控帧，也不算退化
  for (int i = 0; i < 100; ++i) {
    rig.frame(kFailsafe, 16);
  }
  EXPECT_TRUE(rig.status().ch18_unavailable);
  EXPECT_EQ(rig.status().ch18_value, -1);
  EXPECT_FALSE(rig.status().ch18_low_now);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);

  rig.frame(kNormal);
  EXPECT_FALSE(rig.status().ch18_unavailable);
}

TEST(LinkMonitorTest, DisabledSourcesAreIgnored) {
  LinkMonitorConfig config = makeConfig();
  config.enable_ch18 = false;
  config.enable_rc_loss = false;
  Rig rig(config);
  EXPECT_FALSE(rig.status().ch18_unavailable);

  rig.frames(60, kFailsafe);
  EXPECT_FALSE(rig.status().ch18_low_now);
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);

  for (int i = 0; i < 20; ++i) {
    rig.tick();
  }
  EXPECT_FALSE(rig.status().lost);
  EXPECT_FALSE(rig.status().degraded);
}

TEST(LinkMonitorTest, ResultDoesNotDependOnTicks) {
  // 只有RC帧、没有tick时，漏桶按帧积分，确认时刻相同
  LinkMonitor link(makeConfig());
  for (int k = 0; k < 20; ++k) {
    link.onRcFrame(k * kDt, makeFrame(kNormal));
  }
  for (int k = 20; k < 40; ++k) {
    link.onRcFrame(k * kDt, makeFrame(kFailsafe));
  }
  EXPECT_FALSE(link.status().lost);
  link.onRcFrame(40 * kDt, makeFrame(kFailsafe));
  EXPECT_TRUE(link.status().lost);
}

}  // namespace
}  // namespace px4_backtrack::domain
