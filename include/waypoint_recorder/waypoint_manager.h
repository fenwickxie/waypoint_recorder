#ifndef WAYPOINT_RECORDER_WAYPOINT_MANAGER_H
#define WAYPOINT_RECORDER_WAYPOINT_MANAGER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>
#include "waypoint_recorder/waypoint_types.h"

namespace waypoint_recorder {

class WaypointManager {
public:
  WaypointManager() = default;

  void addWaypoint(double x, double y, double z, double yaw, double stamp) {
    int next_id = waypoints_.size() + 1;
    waypoints_.emplace_back(next_id, x, y, z, yaw, stamp);
  }

  bool undoLastWaypoint() {
    if (waypoints_.empty()) {
      return false;
    }
    waypoints_.pop_back();
    return true;
  }

  void clearWaypoints() {
    waypoints_.clear();
  }

  const std::vector<Waypoint>& getWaypoints() const {
    return waypoints_;
  }

  size_t size() const {
    return waypoints_.size();
  }

  bool empty() const {
    return waypoints_.empty();
  }

  const Waypoint* getWaypoint(size_t index) const {
    if (index < waypoints_.size()) {
      return &waypoints_[index];
    }
    return nullptr;
  }

  // 保存航点列表至文件 (简易可移植的 YAML 格式)
  bool saveToFile(const std::string& file_path) const {
    std::ofstream out(file_path);
    if (!out.is_open()) {
      ROS_ERROR_STREAM("[WaypointManager] Failed to open file for writing: " << file_path);
      return false;
    }

    out << "# Waypoint list recorded by waypoint_recorder\n";
    out << "count: " << waypoints_.size() << "\n";
    out << "waypoints:\n";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& wp = waypoints_[i];
      out << "  - id: " << wp.id << "\n";
      out << "    x: " << std::fixed << std::setprecision(4) << wp.x << "\n";
      out << "    y: " << std::fixed << std::setprecision(4) << wp.y << "\n";
      out << "    z: " << std::fixed << std::setprecision(4) << wp.z << "\n";
      out << "    yaw: " << std::fixed << std::setprecision(4) << wp.yaw << "\n";
      out << "    stamp: " << std::fixed << std::setprecision(3) << wp.stamp << "\n";
    }
    out.close();
    ROS_INFO_STREAM("[WaypointManager] Saved " << waypoints_.size() << " waypoints to " << file_path);
    return true;
  }

  // 从文件加载航点列表 (兼容 YAML 键值解析)
  bool loadFromFile(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in.is_open()) {
      ROS_ERROR_STREAM("[WaypointManager] Failed to open file for reading: " << file_path);
      return false;
    }

    std::vector<Waypoint> loaded;
    std::string line;
    Waypoint current_wp;
    bool in_waypoint = false;

    while (std::getline(in, line)) {
      // 去除前后空格
      size_t first = line.find_first_not_of(" \t\r\n");
      if (first == std::string::npos || line[first] == '#') {
        continue;
      }
      std::string trimmed = line.substr(first);

      if (trimmed.find("- id:") == 0) {
        if (in_waypoint) {
          loaded.push_back(current_wp);
        }
        in_waypoint = true;
        current_wp = Waypoint();
        std::stringstream ss(trimmed.substr(5));
        ss >> current_wp.id;
      } else if (in_waypoint) {
        auto parse_field = [&](const std::string& prefix, double& out_val) {
          if (trimmed.find(prefix) == 0) {
            std::stringstream ss(trimmed.substr(prefix.length()));
            ss >> out_val;
            return true;
          }
          return false;
        };

        if (parse_field("x:", current_wp.x)) {}
        else if (parse_field("y:", current_wp.y)) {}
        else if (parse_field("z:", current_wp.z)) {}
        else if (parse_field("yaw:", current_wp.yaw)) {}
        else if (parse_field("stamp:", current_wp.stamp)) {}
      }
    }

    if (in_waypoint) {
      loaded.push_back(current_wp);
    }

    in.close();

    if (loaded.empty()) {
      ROS_WARN_STREAM("[WaypointManager] No waypoints found in file: " << file_path);
      return false;
    }

    // 重构自增 ID
    for (size_t i = 0; i < loaded.size(); ++i) {
      loaded[i].id = static_cast<int>(i + 1);
    }

    waypoints_ = loaded;
    ROS_INFO_STREAM("[WaypointManager] Successfully loaded " << waypoints_.size() << " waypoints from " << file_path);
    return true;
  }

private:
  std::vector<Waypoint> waypoints_;
};

} // namespace waypoint_recorder

#endif // WAYPOINT_RECORDER_WAYPOINT_MANAGER_H
