#include <gtest/gtest.h>

#include "px4_backtrack/domain/link_monitor.hpp"
#include "px4_backtrack/domain/pilot_input.hpp"

namespace px4_backtrack::domain {
namespace {

constexpr double kDt = 0.05;  // RC帧与tick都是20Hz
constexpr uint16_t kReleased = 1900;
constexpr uint16_t kPressed = 1100;
constexpr uint16_t kNormal = LinkMonitor::kCh18NormalValue;
constexpr uint16_t kFailsafe = LinkMonitor::kCh18FailsafeValue;

LinkMonitorConfig makeLinkConfig() {
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

PilotInputConfig makePilotConfig() {
  PilotInputConfig config{};
  config.enable_ch7 = true;
  config.ch7_pressed_threshold = 1500;
  config.ch7_hold_s = 2.0;
  config.ch7_exit_min_s = 0.3;
  config.rc_loss_timeout_s = 0.15;
  return config;
}

/// 按20Hz驱动链路监测和飞手输入：每一步先收一帧，再tick一次
struct Rig {
  explicit Rig(const LinkMonitorConfig& link_config = makeLinkConfig(),
               const PilotInputConfig& pilot_config = makePilotConfig())
    : link(link_config), pilot(pilot_config) {}

  double now() const { return k * kDt; }

  void frame(uint16_t ch7, uint16_t ch18 = kNormal) {
    const double t = now();
    std::vector<uint16_t> channels(18, 1500);
    channels[6] = ch7;
    channels[17] = ch18;
    link.onRcFrame(t, channels);
    pilot.onRcFrame(t, channels, link.status());
    link.update(t);
    pilot.update(t, link.status());
    ++k;
  }

  void frames(int n, uint16_t ch7, uint16_t ch18 = kNormal) {
    for (int i = 0; i < n; ++i) {
      frame(ch7, ch18);
    }
  }

  /// 没有RC帧，只有tick
  void tick() {
    const double t = now();
    link.update(t);
    pilot.update(t, link.status());
    ++k;
  }

  /// 链路正常足够久（超过退化窗口），按钮松开
  void settle() { frames(120, kReleased); }

  const PilotInputStatus& status() const { return pilot.status(); }

  LinkMonitor link;
  PilotInput pilot;
  int k{0};
};

TEST(PilotInputTest, ShortPressProducesExitRequestOnRelease) {
  Rig rig;
  rig.settle();

  const double press_t = rig.now();
  rig.frames(10, kPressed);  // 0.5 s
  EXPECT_TRUE(rig.status().pressed);
  EXPECT_EQ(rig.status().press_id, 1);
  EXPECT_FALSE(rig.status().exit_request.has_value());

  const double release_t = rig.now();
  rig.frame(kReleased);
  EXPECT_FALSE(rig.status().pressed);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 1);
  EXPECT_DOUBLE_EQ(rig.status().exit_request->press_start_t, press_t);
  EXPECT_DOUBLE_EQ(rig.status().exit_request->t, release_t);
  EXPECT_TRUE(rig.status().exit_request->link_ok);
  EXPECT_FALSE(rig.status().return_request.has_value());
}

TEST(PilotInputTest, PressShorterThanExitMinimumIsIgnored) {
  Rig rig;
  rig.settle();

  rig.frames(5, kPressed);  // 0.25 s
  rig.frame(kReleased);
  EXPECT_EQ(rig.status().press_id, 1);
  EXPECT_FALSE(rig.status().exit_request.has_value());

  rig.frames(6, kPressed);  // 0.30 s，正好达到下限
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 2);
}

TEST(PilotInputTest, LongPressProducesReturnRequestOnceAndNoExit) {
  // S8：长按达到 2 s 产生 return_request，松开不产生退出
  Rig rig;
  rig.settle();

  const double press_t = rig.now();
  rig.frames(40, kPressed);  // 1.95 s
  EXPECT_FALSE(rig.status().return_request.has_value());
  rig.frame(kPressed);       // 2.0 s
  ASSERT_TRUE(rig.status().return_request.has_value());
  EXPECT_EQ(rig.status().return_request->press_id, 1);
  EXPECT_DOUBLE_EQ(rig.status().return_request->press_start_t, press_t);
  EXPECT_DOUBLE_EQ(rig.status().return_request->t, press_t + 2.0);
  EXPECT_TRUE(rig.status().return_request->link_ok);

  rig.frames(40, kPressed);
  rig.frame(kReleased);
  EXPECT_FALSE(rig.status().exit_request.has_value());
  ASSERT_TRUE(rig.status().return_request.has_value());
  EXPECT_EQ(rig.status().return_request->press_id, 1);
}

TEST(PilotInputTest, Ch7DipInFailsafeFrameIsNotAPress) {
  // S2：失控瞬间 CH7 单帧低值，与 CH18 低在同一帧
  Rig rig;
  rig.settle();

  rig.frame(kPressed, kFailsafe);
  rig.frames(60, kReleased, kFailsafe);
  rig.frames(60, kReleased);
  EXPECT_EQ(rig.status().press_id, 0);
  EXPECT_FALSE(rig.status().exit_request.has_value());
  EXPECT_FALSE(rig.status().return_request.has_value());
}

TEST(PilotInputTest, PressCutByFailsafeFrameIsCancelled) {
  // S2：CH7 低值出现在第一帧失控帧之前，失控帧到来时作废
  Rig rig;
  rig.settle();

  rig.frame(kPressed);
  EXPECT_TRUE(rig.status().pressed);
  rig.frames(10, kPressed, kFailsafe);
  EXPECT_FALSE(rig.status().pressed);
  rig.frames(60, kReleased);
  EXPECT_EQ(rig.status().press_id, 1);
  EXPECT_FALSE(rig.status().exit_request.has_value());
}

TEST(PilotInputTest, ReleaseAtRecoveryIsNotAShortPress) {
  // S3：失控帧中 CH7 一直为按下（接收机保持），恢复瞬间跳回松开
  Rig rig;
  rig.settle();

  rig.frames(60, kPressed, kFailsafe);
  EXPECT_TRUE(rig.link.status().lost);
  rig.frames(60, kReleased);
  EXPECT_EQ(rig.status().press_id, 0);
  EXPECT_FALSE(rig.status().exit_request.has_value());
}

TEST(PilotInputTest, PressHeldThroughRecoveryDoesNotCount) {
  // S3：恢复时飞手正按着 CH7，这次按下不算，须先松开
  Rig rig;
  rig.settle();

  rig.frames(60, kPressed, kFailsafe);
  rig.frames(10, kPressed);  // 恢复后仍按着 0.5 s
  EXPECT_FALSE(rig.status().pressed);
  rig.frame(kReleased);
  EXPECT_EQ(rig.status().press_id, 0);
  EXPECT_FALSE(rig.status().exit_request.has_value());

  // 一直按着也不会变成长按
  rig.frames(60, kPressed, kFailsafe);
  rig.frames(60, kPressed);
  EXPECT_FALSE(rig.status().return_request.has_value());
}

TEST(PilotInputTest, ShortPressWhileDegradedIsReportedWithLinkNotOk) {
  // t=6.0 的一帧失控帧让链路退化到约 t=11.05；窗口内的短按带 link_ok=false，
  // 由状态机拒绝并告警 EXIT_REJECTED_LINK
  Rig rig;
  rig.settle();

  rig.frame(kReleased, kFailsafe);
  rig.frames(20, kReleased);
  EXPECT_TRUE(rig.link.status().degraded);
  rig.frames(10, kPressed);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 1);
  EXPECT_FALSE(rig.status().exit_request->link_ok);

  // 按下时仍在退化窗口内、松开前窗口已结束：不满足"按下全程"，link_ok 仍为 false
  while (rig.now() < 10.8 + 1e-9) {
    rig.frame(kReleased);
  }
  EXPECT_TRUE(rig.link.status().degraded);
  rig.frames(10, kPressed);  // 10.85 .. 11.3
  EXPECT_FALSE(rig.link.status().degraded);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 2);
  EXPECT_FALSE(rig.status().exit_request->link_ok);

  // 退化窗口结束后按下的短按 link_ok=true
  rig.frames(10, kReleased);
  rig.frames(10, kPressed);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 3);
  EXPECT_TRUE(rig.status().exit_request->link_ok);
}

TEST(PilotInputTest, RcTimeoutCancelsPress) {
  Rig rig;
  rig.settle();

  rig.frames(4, kPressed);
  rig.tick();
  rig.tick();
  rig.tick();  // 距上一帧 0.15 s：RC超时，按下作废
  EXPECT_FALSE(rig.status().pressed);

  // 超时后的帧与之前的帧不相连，不能构成按下沿
  rig.frames(4, kPressed);
  rig.frame(kReleased);
  EXPECT_EQ(rig.status().press_id, 1);
  EXPECT_FALSE(rig.status().exit_request.has_value());
  EXPECT_TRUE(rig.link.status().degraded);
}

TEST(PilotInputTest, FirstFrameCannotStartAPress) {
  Rig rig;
  rig.frames(60, kPressed);
  EXPECT_FALSE(rig.status().pressed);
  EXPECT_EQ(rig.status().press_id, 0);
  EXPECT_FALSE(rig.status().return_request.has_value());

  rig.frame(kReleased);
  rig.frame(kPressed);
  EXPECT_TRUE(rig.status().pressed);
  EXPECT_EQ(rig.status().press_id, 1);
}

TEST(PilotInputTest, PressStartTimeIsReportedForMissionComparison) {
  // S5：飞手按住 CH7 时低电量先触发任务；按下开始时间早于任务开始时间，由状态机拒绝
  Rig rig;
  rig.settle();

  const double press_t = rig.now();
  rig.frames(20, kPressed);  // 期间任务开始
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_DOUBLE_EQ(rig.status().exit_request->press_start_t, press_t);
  EXPECT_TRUE(rig.status().exit_request->link_ok);
}

TEST(PilotInputTest, CommandsAreKeptUntilSuperseded) {
  // R-56：指令保留到同类的下一条，状态机不会因为 tick 相位错过
  Rig rig;
  rig.settle();

  rig.frames(10, kPressed);
  rig.frame(kReleased);
  rig.frames(200, kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 1);

  // 长按不覆盖短按
  rig.frames(50, kPressed);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().return_request.has_value());
  EXPECT_EQ(rig.status().return_request->press_id, 2);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 1);

  rig.frames(10, kPressed);
  rig.frame(kReleased);
  EXPECT_EQ(rig.status().exit_request->press_id, 3);
}

TEST(PilotInputTest, ResetCancelsPressAndClearsCommandsButKeepsIds) {
  // R-57：新飞行周期作废进行中的按下、清除指令，press_id 不归零
  Rig rig;
  rig.settle();

  rig.frames(10, kPressed);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  rig.frames(10, kPressed);
  EXPECT_TRUE(rig.status().pressed);
  EXPECT_EQ(rig.status().press_id, 2);

  rig.pilot.reset();
  EXPECT_FALSE(rig.status().pressed);
  EXPECT_FALSE(rig.status().exit_request.has_value());
  EXPECT_FALSE(rig.status().return_request.has_value());

  // 复位时按着的这次按下不再计时
  rig.frames(60, kPressed);
  EXPECT_FALSE(rig.status().return_request.has_value());
  rig.frame(kReleased);
  EXPECT_FALSE(rig.status().exit_request.has_value());

  rig.frames(10, kPressed);
  rig.frame(kReleased);
  ASSERT_TRUE(rig.status().exit_request.has_value());
  EXPECT_EQ(rig.status().exit_request->press_id, 3);
}

TEST(PilotInputTest, LongPressWhileLinkLostCarriesLinkNotOk) {
  // 恢复期内链路仍判定为丢失，此时达到的长按带 link_ok=false
  LinkMonitorConfig link_config = makeLinkConfig();
  link_config.link_recover_s = 3.0;
  Rig rig(link_config);
  rig.settle();

  rig.frames(30, kReleased, kFailsafe);
  ASSERT_TRUE(rig.link.status().lost);
  rig.frame(kReleased);
  rig.frames(41, kPressed);  // 恢复后 0.05 s 按下，2.0 s 时链路仍未恢复
  ASSERT_TRUE(rig.status().return_request.has_value());
  EXPECT_TRUE(rig.link.status().lost);
  EXPECT_FALSE(rig.status().return_request->link_ok);
}

TEST(PilotInputTest, MissingCh7IsTreatedAsReleased) {
  Rig rig;
  for (int i = 0; i < 60; ++i) {
    const double t = rig.now();
    const std::vector<uint16_t> channels(5, 1000);
    rig.link.onRcFrame(t, channels);
    rig.pilot.onRcFrame(t, channels, rig.link.status());
    ++rig.k;
  }
  EXPECT_FALSE(rig.status().pressed);
  EXPECT_FALSE(rig.status().wait_release);
}

TEST(PilotInputTest, DisabledProducesNothing) {
  PilotInputConfig pilot_config = makePilotConfig();
  pilot_config.enable_ch7 = false;
  Rig rig(makeLinkConfig(), pilot_config);
  rig.settle();
  rig.frames(10, kPressed);
  rig.frame(kReleased);
  rig.frames(60, kPressed);
  EXPECT_EQ(rig.status().press_id, 0);
  EXPECT_FALSE(rig.status().exit_request.has_value());
  EXPECT_FALSE(rig.status().return_request.has_value());
}

}  // namespace
}  // namespace px4_backtrack::domain
