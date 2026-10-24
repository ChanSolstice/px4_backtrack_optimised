#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include <ros/console.h>
#include <fstream>
#include <iomanip>
#include <utility>

namespace px4_backtrack {
namespace infrastructure {

DebugDumper::DebugDumper(std::string dir) : dir_(std::move(dir)) {}

void DebugDumper::dumpInput(int plan, const std::vector<domain::RecordPoint>& points) const {
  const std::string path = dir_ + "/plan_" + std::to_string(plan) + "_input.csv";
  std::ofstream ofs(path);
  ofs << std::fixed << std::setprecision(6) << "# timestamp_s,x_m,y_m,z_m,yaw_rad\n";
  for (const auto& pt : points) {
    ofs << pt.timestamp << "," << pt.position.x() << "," << pt.position.y() << ","
        << pt.position.z() << "," << pt.yaw << "\n";
  }
  if (!ofs) {
    ROS_ERROR("[DebugDumper] Failed to write %s", path.c_str());
    return;
  }
  ROS_INFO("[DebugDumper] Planning input (%zu pts) -> %s", points.size(), path.c_str());
}

void DebugDumper::dumpTrajectory(
    int plan, const std::vector<trajectory_core::TrajectoryState>& states) const {
  const std::string path = dir_ + "/plan_" + std::to_string(plan) + "_trajectory.csv";
  std::ofstream ofs(path);
  ofs << std::fixed << std::setprecision(6)
      << "# t_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,yaw_rad,yaw_rate_rad_s\n";
  for (const auto& s : states) {
    ofs << s.t << "," << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
        << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << "," << s.yaw
        << "," << s.yaw_rate << "\n";
  }
  if (!ofs) {
    ROS_ERROR("[DebugDumper] Failed to write %s", path.c_str());
    return;
  }
  ROS_INFO("[DebugDumper] Trajectory (%zu pts) -> %s", states.size(), path.c_str());
}

}  // namespace infrastructure
}  // namespace px4_backtrack
