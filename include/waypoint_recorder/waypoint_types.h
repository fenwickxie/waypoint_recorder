#ifndef WAYPOINT_RECORDER_WAYPOINT_TYPES_H
#define WAYPOINT_RECORDER_WAYPOINT_TYPES_H

#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace waypoint_recorder {

enum class NavState {
  IDLE,       // 空闲/就绪状态
  NAVIGATING, // 正在执行航点连续发布导航
  PAUSED,     // 暂停发布
  COMPLETED   // 所有航点已到达
};

inline std::string navStateToString(NavState state) {
  switch (state) {
    case NavState::IDLE: return "IDLE";
    case NavState::NAVIGATING: return "NAVIGATING";
    case NavState::PAUSED: return "PAUSED";
    case NavState::COMPLETED: return "COMPLETED";
    default: return "UNKNOWN";
  }
}

struct Waypoint {
  int id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0; // 偏航角(弧度)
  double stamp = 0.0;

  Waypoint() = default;
  Waypoint(int id, double x, double y, double z, double yaw = 0.0, double stamp = 0.0)
      : id(id), x(x), y(y), z(z), yaw(yaw), stamp(stamp) {}
};

} // namespace waypoint_recorder

#endif // WAYPOINT_RECORDER_WAYPOINT_TYPES_H
