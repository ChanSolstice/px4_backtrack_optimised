#include <gtest/gtest.h>

#include "px4_backtrack/domain/trigger_evaluator.hpp"

namespace px4_backtrack::domain {
namespace {

std::vector<uint16_t> makeChannels(uint16_t default_value = 1500) {
  return std::vector<uint16_t>(18, default_value);
}

TriggerConfig makeTriggerConfig() {
  TriggerConfig config{};
  config.enable_ch7 = true;
  config.ch7_pressed_threshold = 1500;
  config.ch7_hold_s = 2.0;
  config.enable_ch18 = true;
  config.ch18_failsafe_threshold = 900;
  config.ch18_confirm_s = 1.0;
  config.enable_rc_loss = true;
  config.rc_loss_timeout_s = 0.15;
  config.rc_loss_debounce_s = 0.1;
  return config;
}

TEST(TriggerEvaluatorTest, Ch7PressEventRequiresStateUpdateAndLongPressTriggersOnce) {
  TriggerConfig config = makeTriggerConfig();
  config.ch7_hold_s = 1.0;
  TriggerEvaluator evaluator(config);

  auto channels = makeChannels();
  channels[6] = 1200;  // pressed
  evaluator.updateRC(channels);
  evaluator.updateNow(0.0);

  EXPECT_FALSE(evaluator.checkCh7PressEvent());
  EXPECT_FALSE(evaluator.checkCh7LongPress());
  EXPECT_TRUE(evaluator.checkCh7PressEvent());

  evaluator.updateNow(0.5);
  EXPECT_FALSE(evaluator.checkCh7LongPress());

  evaluator.updateNow(1.0);
  EXPECT_TRUE(evaluator.checkCh7LongPress());

  evaluator.updateNow(1.5);
  EXPECT_FALSE(evaluator.checkCh7LongPress());
}

TEST(TriggerEvaluatorTest, Ch18FailsafeConfirmsAfterConfiguredDurationAndResets) {
  TriggerConfig config = makeTriggerConfig();
  config.ch18_confirm_s = 1.0;
  TriggerEvaluator evaluator(config);

  auto channels = makeChannels();
  channels[17] = 800;  // failsafe active
  evaluator.updateRC(channels);

  evaluator.updateNow(0.0);
  EXPECT_FALSE(evaluator.checkCh18Failsafe());

  evaluator.updateNow(0.5);
  EXPECT_FALSE(evaluator.checkCh18Failsafe());

  evaluator.updateNow(1.0);
  EXPECT_TRUE(evaluator.checkCh18Failsafe());

  evaluator.updateNow(1.5);
  EXPECT_FALSE(evaluator.checkCh18Failsafe());

  channels[17] = 1500;  // recovered
  evaluator.updateRC(channels);
  evaluator.updateNow(2.0);
  EXPECT_FALSE(evaluator.checkCh18Failsafe());

  channels[17] = 700;  // trigger again
  evaluator.updateRC(channels);
  evaluator.updateNow(2.1);
  EXPECT_FALSE(evaluator.checkCh18Failsafe());
  evaluator.updateNow(3.1);
  EXPECT_TRUE(evaluator.checkCh18Failsafe());
}

TEST(TriggerEvaluatorTest, RCLossEventHasEdgeSemanticsWhileLevelRemainsLatched) {
  TriggerConfig config = makeTriggerConfig();
  config.rc_loss_timeout_s = 0.15;
  config.rc_loss_debounce_s = 0.1;
  TriggerEvaluator evaluator(config);

  evaluator.updateLastRcMsgTime(0.0);

  evaluator.updateNow(0.20);
  EXPECT_FALSE(evaluator.checkRCLossEvent());
  EXPECT_FALSE(evaluator.isRCLost());

  evaluator.updateNow(0.31);
  EXPECT_TRUE(evaluator.checkRCLossEvent());
  EXPECT_TRUE(evaluator.isRCLost());

  evaluator.updateNow(0.40);
  EXPECT_FALSE(evaluator.checkRCLossEvent());
  EXPECT_TRUE(evaluator.isRCLost());

  evaluator.updateLastRcMsgTime(0.41);
  EXPECT_FALSE(evaluator.isRCLost());
}

}  // namespace
}  // namespace px4_backtrack::domain
