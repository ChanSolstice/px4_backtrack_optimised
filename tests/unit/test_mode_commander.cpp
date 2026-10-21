#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "px4_backtrack/application/mode_commander.hpp"

namespace px4_backtrack::application {
namespace {

ModeCommanderConfig makeConfig() {
  ModeCommanderConfig config;
  config.request_interval_s = 1.0;
  config.request_timeout_s = 3.0;
  return config;
}

std::vector<std::string> eventNames(ModeCommander& commander) {
  std::vector<std::string> names;
  for (const auto& event : commander.takeEvents()) {
    names.push_back(event.name);
  }
  return names;
}

TEST(ModeCommanderTest, SendsOnNewIntentThenResendsAtInterval) {
  ModeCommander commander(makeConfig());

  EXPECT_FALSE(commander.update(0.0, "", "POSCTL", true));
  auto sent = commander.update(0.1, "OFFBOARD", "POSCTL", true);
  ASSERT_TRUE(sent);
  EXPECT_EQ(*sent, "OFFBOARD");
  EXPECT_EQ(commander.status().result, ModeRequestResult::Pending);
  EXPECT_DOUBLE_EQ(commander.status().sent_at, 0.1);

  EXPECT_FALSE(commander.update(0.6, "OFFBOARD", "POSCTL", true));
  EXPECT_TRUE(commander.update(1.1, "OFFBOARD", "POSCTL", true));
  EXPECT_FALSE(commander.update(1.5, "OFFBOARD", "POSCTL", true));
  EXPECT_TRUE(commander.update(2.1, "OFFBOARD", "POSCTL", true));
}

TEST(ModeCommanderTest, ObservedModeStopsSending) {
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "POSCTL", true));

  EXPECT_FALSE(commander.update(0.5, "OFFBOARD", "OFFBOARD", true));
  EXPECT_EQ(commander.status().result, ModeRequestResult::Applied);
  EXPECT_FALSE(commander.update(5.0, "OFFBOARD", "OFFBOARD", true));
  EXPECT_FALSE(commander.failed());
}

TEST(ModeCommanderTest, AlreadyInTargetModeDoesNotSend) {
  ModeCommander commander(makeConfig());
  EXPECT_FALSE(commander.update(0.0, "AUTO.LAND", "AUTO.LAND", true));
  EXPECT_EQ(commander.status().result, ModeRequestResult::Applied);
}

TEST(ModeCommanderTest, TimeoutFailsAndKeepsResendingUntilObserved) {
  // S28：飞控拒绝 OFFBOARD，3 s 内告警，按间隔重发
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "POSCTL", true));
  commander.update(1.0, "OFFBOARD", "POSCTL", true);
  commander.update(2.0, "OFFBOARD", "POSCTL", true);
  EXPECT_FALSE(commander.failed());

  EXPECT_TRUE(commander.update(3.0, "OFFBOARD", "POSCTL", true));
  EXPECT_TRUE(commander.failed());
  EXPECT_TRUE(commander.update(4.0, "OFFBOARD", "POSCTL", true));
  EXPECT_TRUE(commander.failed());

  // 观察到目标模式时清除失败
  EXPECT_FALSE(commander.update(4.5, "OFFBOARD", "OFFBOARD", true));
  EXPECT_FALSE(commander.failed());
}

TEST(ModeCommanderTest, NotSentResultFailsImmediately) {
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "POSCTL", true));

  // 旧意图的结果不影响当前意图
  commander.onResult(0.05, "POSCTL", false);
  EXPECT_FALSE(commander.failed());

  commander.onResult(0.05, "OFFBOARD", true);
  EXPECT_FALSE(commander.failed());
  commander.onResult(0.10, "OFFBOARD", false);
  EXPECT_TRUE(commander.failed());
  EXPECT_TRUE(commander.update(1.0, "OFFBOARD", "POSCTL", true));
}

TEST(ModeCommanderTest, IntentChangeRestartsTimingAndClearsFailure) {
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "POSCTL", true));
  commander.update(3.0, "OFFBOARD", "POSCTL", true);
  ASSERT_TRUE(commander.failed());

  auto sent = commander.update(3.05, "AUTO.LAND", "POSCTL", true);
  ASSERT_TRUE(sent);
  EXPECT_EQ(*sent, "AUTO.LAND");
  EXPECT_FALSE(commander.failed());
  commander.update(5.9, "AUTO.LAND", "POSCTL", true);
  EXPECT_FALSE(commander.failed());
  commander.update(6.05, "AUTO.LAND", "POSCTL", true);
  EXPECT_TRUE(commander.failed());

  // 意图消失：不发送，告警清除
  EXPECT_FALSE(commander.update(6.1, "", "POSCTL", true));
  EXPECT_FALSE(commander.failed());
  EXPECT_EQ(commander.status().result, ModeRequestResult::None);
}

TEST(ModeCommanderTest, ModeLostAfterAppliedStartsNewRound) {
  // 生效后被切走（例如重新申请控制权）：立即重发并重新计时
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "POSCTL", true));
  commander.update(0.5, "OFFBOARD", "OFFBOARD", true);
  ASSERT_EQ(commander.status().result, ModeRequestResult::Applied);

  EXPECT_TRUE(commander.update(10.0, "OFFBOARD", "POSCTL", true));
  EXPECT_EQ(commander.status().result, ModeRequestResult::Pending);
  commander.update(12.9, "OFFBOARD", "POSCTL", true);
  EXPECT_FALSE(commander.failed());
  commander.update(13.0, "OFFBOARD", "POSCTL", true);
  EXPECT_TRUE(commander.failed());
}

TEST(ModeCommanderTest, StaleFcuStateCannotConfirm) {
  // 飞控状态中断时无法确认生效，只按间隔重发；生效后中断不重发
  ModeCommander commander(makeConfig());
  ASSERT_TRUE(commander.update(0.0, "OFFBOARD", "OFFBOARD", false));
  EXPECT_EQ(commander.status().result, ModeRequestResult::Pending);
  EXPECT_TRUE(commander.update(1.0, "OFFBOARD", "OFFBOARD", false));

  commander.update(1.5, "OFFBOARD", "OFFBOARD", true);
  ASSERT_EQ(commander.status().result, ModeRequestResult::Applied);
  EXPECT_FALSE(commander.update(5.0, "OFFBOARD", "POSCTL", false));
  EXPECT_EQ(commander.status().result, ModeRequestResult::Applied);
}

TEST(ModeCommanderTest, EventsTraceIntentSendAndResult) {
  ModeCommander commander(makeConfig());
  commander.update(0.0, "OFFBOARD", "POSCTL", true);
  commander.update(0.5, "OFFBOARD", "OFFBOARD", true);
  EXPECT_EQ(eventNames(commander),
            (std::vector<std::string>{"intent", "sent", "applied"}));

  commander.update(1.0, "AUTO.LAND", "OFFBOARD", true);
  commander.onResult(1.05, "AUTO.LAND", false);
  EXPECT_EQ(eventNames(commander),
            (std::vector<std::string>{"intent", "sent", "failed"}));
  EXPECT_TRUE(eventNames(commander).empty());
}

}  // namespace
}  // namespace px4_backtrack::application
