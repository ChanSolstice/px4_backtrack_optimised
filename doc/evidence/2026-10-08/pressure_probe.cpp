#include "px4_backtrack/adapters/diagnostics_publisher.hpp"
#include "px4_backtrack/adapters/debug_publishers.hpp"
#include "px4_backtrack/application/planning_worker.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <atomic>
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
  ros::init(argc, argv, "scale_backtrack");
  ros::NodeHandle nh, private_nh("~");
  px4_backtrack::adapters::DiagnosticsPublisher publisher(nh);
  px4_backtrack::adapters::DebugPublisher debug_publisher(nh);
  publisher.publishConfiguration(private_nh);
  auto cost_pub = nh.advertise<std_msgs::Float64>("/probe/phase_cost_ms", 1);
  auto ready_pub = nh.advertise<std_msgs::String>("/probe/ready", 1, true);
  std::atomic<int> requested{0};
  auto command_sub = nh.subscribe<std_msgs::String>("/probe/command", 1,
      [&](const std_msgs::String::ConstPtr &command) {
        requested.store(command->data == "started" ? 1 : 2);
      });
  std::vector<px4_backtrack::domain::RecordPoint> points;
  points.reserve(54000);
  for (int i = 0; i < 54000; ++i)
    points.emplace_back(px4_backtrack::domain::Vec3(i * 0.00005, 0.0, 1.0),
                        0.123 + i * 0.000001, 1000.0 + i / 15.0);
  trajectory_core::TrajectoryConfig config;
  config.v_max = 0.6;
  px4_backtrack::application::PlanningTrace trace;
  trace.flight_id = 1;
  trace.plan_id = 1;
  trace.points = &points;
  trace.config = &config;
  uint64_t sequence = 0;
  double previous_tick = -1;
  auto timer = nh.createWallTimer(ros::WallDuration(0.05), [&](const ros::WallTimerEvent &) {
    const double now = ros::SteadyTime::now().toSec();
    const double interval = previous_tick < 0 ? 0 : now - previous_tick;
    previous_tick = now;
    px4_backtrack::application::ControlDiagnostics snapshot;
    snapshot.flight_id = 1;
    snapshot.plan_id = 1;
    snapshot.steady_time = now;
    snapshot.inputs_fresh = snapshot.odom_valid = true;
    snapshot.armed = true;
    snapshot.current_state = px4_backtrack::domain::State::ReturnActive;
    snapshot.fcu_mode = "OFFBOARD";
    snapshot.actual_position = px4_backtrack::domain::Vec3(1.0, 0.0, 1.0);
    snapshot.trajectory_valid = snapshot.projection_valid = true;
    snapshot.route_projection_valid = snapshot.route_prediction_valid = true;
    snapshot.route_safety_evaluation_time = now;
    snapshot.route_distance = 0.01;
    snapshot.route_predicted_deviation = 0.06;
    snapshot.recorded_points = 54000;
    snapshot.should_publish_setpoint = true;
    snapshot.setpoint = px4_backtrack::domain::SetpointTarget();
    snapshot.setpoint->position = snapshot.actual_position;
    snapshot.setpoint->type_mask = 0x09F8;
    publisher.publishControl(snapshot, ++sequence, interval, ros::SteadyTime::now().toSec() - now);
  });
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std_msgs::String ready;
  ready.data = "ready";
  ready_pub.publish(ready);
  while (ros::ok()) {
    const int phase = requested.exchange(0);
    if (phase) {
      trace.phase = phase == 1 ? "started" : "finished";
      trace.success = phase == 2;
      trace.generation.status = phase == 2 ? "OK" : "NOT_ATTEMPTED";
      trace.generation.report.profile.path_length_m = 2.69995;
      trace.generation.report.profile.total_duration_s = 12.0;
      const double start = ros::SteadyTime::now().toSec();
      publisher.publishPlanning(trace);
      std_msgs::Float64 cost;
      cost.data = 1000.0 * (ros::SteadyTime::now().toSec() - start);
      cost_pub.publish(cost);
      if (phase == 2) {
        debug_publisher.publishWaypoints(points);
        std::vector<trajectory_core::TrajectoryState> states(20000);
        for (size_t i = 0; i < states.size(); ++i) {
          states[i].t = i * 0.1;
          states[i].position = px4_backtrack::domain::Vec3(i * 2.69995 / 19999.0, 0.0, 1.0);
          states[i].yaw = 0.123;
        }
        debug_publisher.publishTrajectory(states);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 0;
}
