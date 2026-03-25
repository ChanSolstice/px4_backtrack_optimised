#include "px4_backtrack/adapters/debug_publishers.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>

namespace px4_backtrack {
namespace adapters {

DebugPublisher::DebugPublisher(ros::NodeHandle& nh) {
  // latched = true: rviz 连接后也能看到最近一次发布
  waypoints_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/px4_backtrack/debug/waypoints", 1, /*latch=*/true);
  trajectory_pub_ = nh.advertise<nav_msgs::Path>(
      "/px4_backtrack/debug/trajectory", 1, /*latch=*/true);
  ROS_INFO("[DebugPublisher] Initialized: /px4_backtrack/debug/waypoints, /px4_backtrack/debug/trajectory");
}

void DebugPublisher::publishWaypoints(const std::vector<domain::RecordPoint>& points) {
  if (points.empty()) return;

  visualization_msgs::MarkerArray markers;
  ros::Time stamp = ros::Time::now();

  // 球体标记: 每个录制点
  for (size_t i = 0; i < points.size(); ++i) {
    visualization_msgs::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "map";
    m.ns = "recorded_waypoints";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = points[i].position.x();
    m.pose.position.y = points[i].position.y();
    m.pose.position.z = points[i].position.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.12;
    m.scale.y = 0.12;
    m.scale.z = 0.12;
    // 绿色
    m.color.r = 0.2f;
    m.color.g = 0.8f;
    m.color.b = 0.2f;
    m.color.a = 0.8f;
    m.lifetime = ros::Duration(0);  // 永久
    markers.markers.push_back(m);
  }

  // 连线: 串联所有录制点
  {
    visualization_msgs::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = "map";
    line.ns = "recorded_waypoints";
    line.id = static_cast<int>(points.size());
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.04;  // 线宽
    line.color.r = 0.2f;
    line.color.g = 0.8f;
    line.color.b = 0.2f;
    line.color.a = 0.5f;
    line.lifetime = ros::Duration(0);

    for (const auto& pt : points) {
      geometry_msgs::Point p;
      p.x = pt.position.x();
      p.y = pt.position.y();
      p.z = pt.position.z();
      line.points.push_back(p);
    }
    markers.markers.push_back(line);
  }

  // 起点标记 (大红球)
  {
    visualization_msgs::Marker start;
    start.header.stamp = stamp;
    start.header.frame_id = "map";
    start.ns = "recorded_waypoints";
    start.id = static_cast<int>(points.size()) + 1;
    start.type = visualization_msgs::Marker::SPHERE;
    start.action = visualization_msgs::Marker::ADD;
    start.pose.position.x = points.front().position.x();
    start.pose.position.y = points.front().position.y();
    start.pose.position.z = points.front().position.z();
    start.pose.orientation.w = 1.0;
    start.scale.x = 0.25;
    start.scale.y = 0.25;
    start.scale.z = 0.25;
    start.color.r = 1.0f;
    start.color.g = 0.0f;
    start.color.b = 0.0f;
    start.color.a = 1.0f;
    start.lifetime = ros::Duration(0);
    markers.markers.push_back(start);
  }

  // 终点标记 (大蓝球)
  {
    visualization_msgs::Marker end;
    end.header.stamp = stamp;
    end.header.frame_id = "map";
    end.ns = "recorded_waypoints";
    end.id = static_cast<int>(points.size()) + 2;
    end.type = visualization_msgs::Marker::SPHERE;
    end.action = visualization_msgs::Marker::ADD;
    end.pose.position.x = points.back().position.x();
    end.pose.position.y = points.back().position.y();
    end.pose.position.z = points.back().position.z();
    end.pose.orientation.w = 1.0;
    end.scale.x = 0.25;
    end.scale.y = 0.25;
    end.scale.z = 0.25;
    end.color.r = 0.0f;
    end.color.g = 0.3f;
    end.color.b = 1.0f;
    end.color.a = 1.0f;
    end.lifetime = ros::Duration(0);
    markers.markers.push_back(end);
  }

  waypoints_pub_.publish(markers);
  ROS_INFO("[DebugPublisher] Published %zu waypoint markers", points.size());
}

void DebugPublisher::publishTrajectory(
    const std::vector<trajectory_core::TrajectoryState>& states) {
  if (states.empty()) return;

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  for (const auto& s : states) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = s.position.x();
    pose.pose.position.y = s.position.y();
    pose.pose.position.z = s.position.z();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path.poses.push_back(pose);
  }

  trajectory_pub_.publish(path);
  ROS_INFO("[DebugPublisher] Published trajectory path (%zu poses)", states.size());
}

}  // namespace adapters
}  // namespace px4_backtrack
