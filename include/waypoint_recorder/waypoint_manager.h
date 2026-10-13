#ifndef WAYPOINT_RECORDER_WAYPOINT_MANAGER_H
#define WAYPOINT_RECORDER_WAYPOINT_MANAGER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cctype>
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

  // 根据文件扩展名保存为 YAML 或 ASCII PLY。
  bool saveToFile(const std::string& file_path) const {
    if (hasPlyExtension(file_path)) {
      return saveToPlyFile(file_path);
    }

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

  // 根据文件扩展名加载 YAML 或 ASCII PLY 航点列表。
  bool loadFromFile(const std::string& file_path) {
    if (hasPlyExtension(file_path)) {
      return loadFromPlyFile(file_path);
    }

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
  static bool hasPlyExtension(const std::string& file_path) {
    const size_t dot = file_path.find_last_of('.');
    if (dot == std::string::npos) {
      return false;
    }

    std::string extension = file_path.substr(dot);
    for (char& character : extension) {
      character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    }
    return extension == ".ply";
  }

  bool saveToPlyFile(const std::string& file_path) const {
    std::ofstream out(file_path);
    if (!out.is_open()) {
      ROS_ERROR_STREAM("[WaypointManager] Failed to open file for writing: " << file_path);
      return false;
    }

    out << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << waypoints_.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "end_header\n"
        << std::fixed << std::setprecision(6);
    for (const auto& waypoint : waypoints_) {
      out << waypoint.x << " " << waypoint.y << " " << waypoint.z << "\n";
    }

    if (!out.good()) {
      ROS_ERROR_STREAM("[WaypointManager] Failed while writing file: " << file_path);
      return false;
    }
    ROS_INFO_STREAM("[WaypointManager] Saved " << waypoints_.size() << " waypoints to " << file_path);
    return true;
  }

  bool loadFromPlyFile(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in.is_open()) {
      ROS_ERROR_STREAM("[WaypointManager] Failed to open file for reading: " << file_path);
      return false;
    }

    bool is_ascii = false;
    bool found_vertex_element = false;
    bool found_end_header = false;
    size_t vertex_count = 0;
    size_t property_count = 0;
    size_t x_index = static_cast<size_t>(-1);
    size_t y_index = static_cast<size_t>(-1);
    size_t z_index = static_cast<size_t>(-1);
    std::string line;

    while (std::getline(in, line)) {
      std::istringstream header_line(line);
      std::string keyword;
      header_line >> keyword;

      if (keyword == "format") {
        std::string format;
        header_line >> format;
        is_ascii = (format == "ascii");
      } else if (keyword == "element") {
        std::string element_name;
        size_t element_count = 0;
        if (!(header_line >> element_name >> element_count)) {
          continue;
        }
        found_vertex_element = (element_name == "vertex");
        if (found_vertex_element) {
          vertex_count = element_count;
        }
      } else if (keyword == "property" && found_vertex_element) {
        std::string property_type;
        std::string property_name;
        if (!(header_line >> property_type >> property_name) || property_type == "list") {
          ROS_ERROR_STREAM("[WaypointManager] Unsupported PLY vertex property in " << file_path);
          return false;
        }
        if (property_name == "x") x_index = property_count;
        else if (property_name == "y") y_index = property_count;
        else if (property_name == "z") z_index = property_count;
        ++property_count;
      } else if (keyword == "end_header") {
        found_end_header = true;
        break;
      }
    }

    if (!is_ascii || !found_vertex_element || !found_end_header || vertex_count == 0 ||
        x_index == static_cast<size_t>(-1) || y_index == static_cast<size_t>(-1) ||
        z_index == static_cast<size_t>(-1)) {
      ROS_ERROR_STREAM("[WaypointManager] Invalid or unsupported ASCII PLY file: " << file_path);
      return false;
    }

    std::vector<Waypoint> loaded;
    loaded.reserve(vertex_count);
    while (loaded.size() < vertex_count && std::getline(in, line)) {
      std::istringstream vertex_line(line);
      std::vector<double> values;
      double value = 0.0;
      while (vertex_line >> value) {
        values.push_back(value);
      }
      if (values.empty()) {
        continue;
      }
      if (values.size() < property_count) {
        ROS_ERROR_STREAM("[WaypointManager] Invalid vertex data in PLY file: " << file_path);
        return false;
      }
      loaded.emplace_back(static_cast<int>(loaded.size() + 1), values[x_index], values[y_index], values[z_index]);
    }

    if (loaded.size() != vertex_count) {
      ROS_ERROR_STREAM("[WaypointManager] PLY vertex count does not match its data: " << file_path);
      return false;
    }

    waypoints_ = loaded;
    ROS_INFO_STREAM("[WaypointManager] Successfully loaded " << waypoints_.size() << " waypoints from " << file_path);
    return true;
  }

  std::vector<Waypoint> waypoints_;
};

} // namespace waypoint_recorder

#endif // WAYPOINT_RECORDER_WAYPOINT_MANAGER_H
