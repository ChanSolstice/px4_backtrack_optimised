#include "px4_backtrack/infrastructure/debug_dumper.hpp"
#include <ros/console.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

namespace px4_backtrack {
namespace infrastructure {

DebugDumper::DebugDumper(const std::string& output_dir)
  : output_dir_(output_dir)
{
  if (output_dir_.empty()) {
    const char* home = std::getenv("HOME");
    output_dir_ = std::string(home ? home : "/tmp") + "/.ros/backtrack_debug";
  }
}

bool DebugDumper::ensureDirectory() {
  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    ROS_ERROR("[DebugDumper] Failed to create directory '%s': %s",
              output_dir_.c_str(), ec.message().c_str());
    return false;
  }
  return true;
}

std::string DebugDumper::makeFilename(const std::string& prefix) const {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time_t, &tm);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s_%04d%02d%02d_%02d%02d%02d.csv",
                prefix.c_str(),
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);

  return output_dir_ + "/" + buf;
}

std::string DebugDumper::dumpWaypoints(const std::vector<domain::RecordPoint>& points) {
  if (!ensureDirectory()) return "";

  std::string filepath = makeFilename("waypoints");
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    ROS_ERROR("[DebugDumper] Cannot open file: %s", filepath.c_str());
    return "";
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "# timestamp_s,x_m,y_m,z_m,yaw_rad\n";

  for (const auto& pt : points) {
    ofs << pt.timestamp << ","
        << pt.position.x() << ","
        << pt.position.y() << ","
        << pt.position.z() << ","
        << pt.yaw << "\n";
  }

  ofs.close();
  ROS_INFO("[DebugDumper] Waypoints (%zu pts) -> %s", points.size(), filepath.c_str());
  return filepath;
}

std::string DebugDumper::dumpTrajectory(
    const std::vector<trajectory_core::TrajectoryState>& states) {
  if (!ensureDirectory()) return "";

  std::string filepath = makeFilename("trajectory");
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    ROS_ERROR("[DebugDumper] Cannot open file: %s", filepath.c_str());
    return "";
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "# t_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,yaw_rad,yaw_rate_rad_s\n";

  for (const auto& s : states) {
    ofs << s.t << ","
        << s.position.x() << ","
        << s.position.y() << ","
        << s.position.z() << ","
        << s.velocity.x() << ","
        << s.velocity.y() << ","
        << s.velocity.z() << ","
        << s.yaw << ","
        << s.yaw_rate << "\n";
  }

  ofs.close();
  ROS_INFO("[DebugDumper] Trajectory (%zu pts) -> %s", states.size(), filepath.c_str());
  return filepath;
}

}  // namespace infrastructure
}  // namespace px4_backtrack
