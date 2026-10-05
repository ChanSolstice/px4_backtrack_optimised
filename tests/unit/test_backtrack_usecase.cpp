#include <gtest/gtest.h>
#include <limits>
#include "px4_backtrack/application/backtrack_usecase.hpp"
namespace px4_backtrack::application {
namespace {
UseCaseConfig makeConfig() {
  UseCaseConfig c; c.record_delay_s=0; c.record_min_points=3; c.record_max_hz=100;
  c.state_machine_config.pre_offboard_setpoint_count=0;
  c.state_machine_config.offboard_attempt_interval_s=0;
  c.trigger_config.enable_ch7=true; c.trigger_config.ch7_hold_s=0;
  c.trigger_config.ch7_pressed_threshold=1200;
  c.trigger_config.enable_ch18=false; c.trigger_config.enable_rc_loss=false;
  c.trigger_config.enable_low_battery=false; c.trigger_config.low_battery_threshold=0.15;
  return c;
}
struct Flight {
  BacktrackUseCase usecase;
  double now=10.0, stamp=10.0;
  bool armed=false;
  std::string mode="OFFBOARD";
  domain::Vec3 position{0,0,1}, velocity=domain::Vec3::Zero();
  double yaw=std::acos(-1.0);
  explicit Flight(UseCaseConfig c=makeConfig()):usecase(c) {
    step(); armed=true; step(); step();
  }
  TickOutput step(double dt=0.05, bool odom=true, bool state=true) {
    now+=dt; stamp+=dt;
    if (state) usecase.onStateReceived(armed,mode,now);
    if (odom) usecase.onOdomReceived(position,velocity,yaw,stamp,now);
    return usecase.tick(now);
  }
  void recordLine() {
    for(int i=0;i<=20;++i) { position.x()=i*0.1; step(0.2); }
  }
  void rc(uint16_t ch) {
    std::vector<uint16_t> channels(18,1500); channels[6]=ch;
    usecase.onRCReceived(channels,now);
  }
  TickOutput start() { rc(1000); step(); return step(); }
  void release() { rc(1500); step(); }
  TickOutput pause() { release(); rc(1000); return step(); }
  TickOutput resume() { release(); rc(1000); step(); return step(); }
};
TEST(BacktrackUseCaseTest, InvalidConfigurationFallsBackToPositionYawWaypoints) {
  auto c=makeConfig(); c.trajectory_config.v_end_max=2;
  Flight f(c); f.recordLine(); auto out=f.start();
  ASSERT_EQ(out.current_state,domain::State::ReturnActive);
  ASSERT_TRUE(out.setpoint); EXPECT_EQ(out.setpoint->type_mask,0x09F8);
}
TEST(BacktrackUseCaseTest, NoSpatialPathStillLands) {
  Flight f; for(int i=0;i<4;++i) f.step(); auto out=f.start();
  EXPECT_EQ(out.current_state,domain::State::LandingTransition);
}
TEST(BacktrackUseCaseTest, AutoLandYieldPersistsUntilDisarmEvenWithoutOdometry) {
  Flight f; f.recordLine(); ASSERT_EQ(f.start().current_state,domain::State::ReturnActive);
  f.mode="AUTO.LAND"; auto out=f.step(0.05,false);
  EXPECT_EQ(out.current_state,domain::State::AutoLandYield); EXPECT_FALSE(out.mode_request); EXPECT_FALSE(out.setpoint);
  f.mode="POSCTL"; out=f.step(); EXPECT_EQ(out.current_state,domain::State::AutoLandYield);
  EXPECT_FALSE(out.mode_request); EXPECT_FALSE(out.setpoint);
  f.armed=false; EXPECT_EQ(f.step().current_state,domain::State::IdleDisarmed);
}
TEST(BacktrackUseCaseTest, StationaryAircraftDoesNotFinishAfterNominalDuration) {
  Flight f; f.recordLine(); ASSERT_EQ(f.start().current_state,domain::State::ReturnActive);
  TickOutput out; for(int i=0;i<2000;++i) out=f.step();
  EXPECT_EQ(out.current_state,domain::State::ReturnActive); ASSERT_TRUE(out.setpoint);
  EXPECT_LT((out.setpoint->position-f.position).norm(),0.201);
  EXPECT_LT(out.setpoint->velocity.norm(),1e-5);
}
TEST(BacktrackUseCaseTest, StartAndResumeAlignYawBeforeTranslation) {
  Flight f; f.recordLine(); f.yaw=0.0;
  auto out=f.start(); EXPECT_EQ(out.current_state,domain::State::ReturnPreOffboard);
  ASSERT_TRUE(out.setpoint); EXPECT_TRUE(out.setpoint->position.isApprox(f.position));
  f.yaw=std::acos(-1.0); EXPECT_EQ(f.step().current_state,domain::State::ReturnActive);
  EXPECT_EQ(f.pause().current_state,domain::State::Paused);
  f.yaw=0.0; EXPECT_EQ(f.resume().current_state,domain::State::ReturnPreOffboard);
  f.yaw=std::acos(-1.0); EXPECT_EQ(f.step().current_state,domain::State::ReturnActive);
}
TEST(BacktrackUseCaseTest, StaleOdometryFreezesAndRequiresExplicitResume) {
  Flight f; f.recordLine(); f.start(); auto out=f.step(0.30,false);
  EXPECT_EQ(out.current_state,domain::State::Paused); ASSERT_TRUE(out.setpoint);
  EXPECT_DOUBLE_EQ(out.setpoint->velocity.norm(),0.0);
  EXPECT_EQ(out.mode_request,domain::ModeRequest::POSCTL);
  out=f.step(); EXPECT_EQ(out.current_state,domain::State::Paused);
  EXPECT_EQ(f.resume().current_state,domain::State::ReturnActive);
}
TEST(BacktrackUseCaseTest, StaleModeCannotActivateOrIssueModeRequests) {
  Flight f; f.recordLine(); f.start(); auto out=f.step(0.60,true,false);
  EXPECT_EQ(out.current_state,domain::State::Paused); EXPECT_FALSE(out.mode_request);
  ASSERT_TRUE(out.setpoint); EXPECT_DOUBLE_EQ(out.setpoint->velocity.norm(),0);
}
TEST(BacktrackUseCaseTest, NonFiniteInputNeverReachesControlOutput) {
  Flight f; f.recordLine(); f.start();
  f.usecase.onOdomReceived(domain::Vec3::Constant(std::numeric_limits<double>::quiet_NaN()),f.velocity,f.yaw,f.stamp+0.01,f.now+0.01);
  auto out=f.step(0.05,false);
  EXPECT_EQ(out.current_state,domain::State::Paused); ASSERT_TRUE(out.setpoint);
  EXPECT_TRUE(out.setpoint->position.allFinite()); EXPECT_DOUBLE_EQ(out.setpoint->velocity.norm(),0);
}
TEST(BacktrackUseCaseTest, PositionOrStampResetDisablesOldCoordinateTargetsUntilDisarm) {
  Flight f; f.recordLine(); f.start(); f.position.x()+=10;
  auto out=f.step(); EXPECT_EQ(out.current_state,domain::State::Paused); EXPECT_FALSE(out.setpoint);
  out=f.step(); EXPECT_FALSE(out.setpoint);
  f.armed=false; EXPECT_EQ(f.step().current_state,domain::State::IdleDisarmed);
  Flight g; g.recordLine(); g.start(); g.stamp-=2;
  out=g.step(); EXPECT_EQ(out.current_state,domain::State::Paused); EXPECT_FALSE(out.setpoint);
}
TEST(BacktrackUseCaseTest, ThreeDimensionalRouteBudgetStopsNormalAndFallback) {
  for(bool fallback:{false,true}) {
    auto c=makeConfig(); if(fallback)c.trajectory_config.v_end_max=2;
    Flight f(c); f.recordLine(); f.start(); f.position.z()+=0.20; f.velocity.z()=0.3;
    auto out=f.step(); EXPECT_EQ(out.current_state,domain::State::Paused); ASSERT_TRUE(out.setpoint);
    EXPECT_DOUBLE_EQ(out.setpoint->velocity.norm(),0);
    EXPECT_EQ(f.resume().current_state,domain::State::Paused);
  }
}
TEST(BacktrackUseCaseTest, LowBatteryReturnInsufficientPointsAndPausePolicy) {
  auto c=makeConfig(); c.trigger_config.enable_low_battery=true; c.trigger_config.low_battery_confirm_s=0;
  Flight f(c); f.recordLine(); f.usecase.onBatteryReceived(0.10);
  auto out=f.step(); EXPECT_TRUE(out.low_battery); EXPECT_EQ(out.current_state,domain::State::ReturnActive);
  f.pause(); f.rc(1500); out=f.step(); EXPECT_TRUE(out.low_battery); EXPECT_EQ(out.current_state,domain::State::Paused);
  c.record_min_points=100; Flight g(c); g.usecase.onBatteryReceived(0.10);
  out=g.step(); EXPECT_EQ(out.current_state,domain::State::LandingTransition);
  c.record_delay_s=15; Flight h(c); h.usecase.onBatteryReceived(0.10);
  out=h.step(); EXPECT_TRUE(out.low_battery); EXPECT_EQ(out.current_state,domain::State::WaitRecordDelay);
}
TEST(BacktrackUseCaseTest, ExternalOffboardExitYieldsAndOnlyExplicitValidResumeCanContinue) {
  Flight f; f.recordLine(); ASSERT_EQ(f.start().current_state,domain::State::ReturnActive);
  f.mode="POSCTL"; auto out=f.step();
  EXPECT_EQ(out.current_state,domain::State::ControlYield); EXPECT_FALSE(out.mode_request); EXPECT_FALSE(out.setpoint);
  for(int i=0;i<100;++i) { out=f.step(); EXPECT_FALSE(out.mode_request); EXPECT_FALSE(out.setpoint); }
  EXPECT_EQ(f.resume().current_state,domain::State::ReturnPreOffboard);
  f.mode="OFFBOARD"; EXPECT_EQ(f.step().current_state,domain::State::ReturnActive);
}

TEST(BacktrackUseCaseTest, RejectsInvalidEngineeringLimits) {
  auto c=makeConfig(); c.route_braking_m_s2=0;
  EXPECT_THROW(BacktrackUseCase usecase(c),std::invalid_argument);
  c=makeConfig(); c.trajectory_config.v_max=1.0;
  EXPECT_THROW(BacktrackUseCase usecase(c),std::invalid_argument);
}
}
}
