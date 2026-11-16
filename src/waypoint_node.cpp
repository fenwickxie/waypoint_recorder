#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <ros/names.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

#include "waypoint_recorder/waypoint_types.h"
#include "waypoint_recorder/waypoint_manager.h"
#include "waypoint_recorder/WaypointRecorderConfig.h"

namespace waypoint_recorder {

class WaypointNode {
public:
  WaypointNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), current_idx_(0), state_(NavState::IDLE),
        is_waiting_(false), wait_start_time_(0.0), has_odom_(false),
        vehicle_x_(0.0), vehicle_y_(0.0), vehicle_z_(0.0), vehicle_yaw_(0.0),
        last_odom_time_(0.0) {
    loadParameters();
    setupROS();
    setupDynamicReconfigure();
    ROS_INFO("[WaypointNode] Initialized successfully. Waiting for commands and odometry...");
  }

  void update() {
    publishMarkers();
    publishStatus();

    if (state_ != NavState::NAVIGATING) {
      return;
    }

    if (manager_.empty()) {
      ROS_WARN_THROTTLE(2.0, "[WaypointNode] Navigating requested, but waypoint list is empty!");
      state_ = NavState::IDLE;
      return;
    }

    if (!has_odom_) {
      ROS_WARN_THROTTLE(2.0, "[WaypointNode] Waiting for vehicle odometry...");
      return;
    }

    if (current_idx_ >= manager_.size()) {
      if (loop_navigation_) {
        ROS_INFO("[WaypointNode] Completed all waypoints. Loop navigation enabled, returning to first waypoint.");
        current_idx_ = 0;
        publishCurrentGoal();
      } else {
        ROS_INFO("[WaypointNode] Reached all waypoints. Navigation completed.");
        state_ = NavState::COMPLETED;
      }
      return;
    }

    const Waypoint* target = manager_.getWaypoint(current_idx_);
    if (!target) {
      state_ = NavState::IDLE;
      return;
    }

    // 计算当前机器人与当前航点的平面欧几里得距离与垂直高度差
    double dx = vehicle_x_ - target->x;
    double dy = vehicle_y_ - target->y;
    double dz = vehicle_z_ - target->z;
    double dist_xy = std::sqrt(dx * dx + dy * dy);
    double dist_z = std::abs(dz);

    bool is_last_waypoint = (current_idx_ == manager_.size() - 1);

    // 核心平滑切换与到达逻辑：
    // 1. 若开启了提前发布 (enable_early_switch) 且不是最后一个点：
    //    当机器人接近到 early_switch_radius 阈值之内时，提前发布下一个目标点，
    //    避免局部规划器因目标点到达而大幅减速造成停顿或顿挫。
    if (enable_early_switch_ && !is_last_waypoint) {
      if (dist_xy <= early_switch_radius_ && dist_z <= z_bound_) {
        ROS_INFO("[WaypointNode] [Smooth Early Switch] Distance to WP %d (%.2fm) <= early_switch_radius (%.2fm). Advancing to next WP %d directly!",
                 target->id, dist_xy, early_switch_radius_, target->id + 1);
        current_idx_++;
        publishCurrentGoal();
        is_waiting_ = false;
        return;
      }
    }

    // 2. 传统精确到达判定（针对最后一个点，或者当关闭了提前切换功能时）
    if (dist_xy <= arrival_radius_ && dist_z <= z_bound_) {
      if (!is_waiting_) {
        is_waiting_ = true;
        wait_start_time_ = ros::Time::now().toSec();
        ROS_INFO("[WaypointNode] Arrived at WP %d (dist: %.2fm). Waiting for %.2fs...",
                 target->id, dist_xy, wait_time_);
      }

      double current_time = ros::Time::now().toSec();
      if (current_time - wait_start_time_ >= wait_time_) {
        is_waiting_ = false;
        if (is_last_waypoint) {
          if (loop_navigation_) {
            ROS_INFO("[WaypointNode] Reached final WP %d! Looping back to first WP.", target->id);
            current_idx_ = 0;
            publishCurrentGoal();
          } else {
            ROS_INFO("[WaypointNode] Reached final WP %d! Navigation finished.", target->id);
            state_ = NavState::COMPLETED;
          }
        } else {
          current_idx_++;
          ROS_INFO("[WaypointNode] Finished waiting at WP %d. Proceeding to WP %d.",
                   target->id, target->id + 1);
          publishCurrentGoal();
        }
      }
    } else {
      // 周期性持续发布当前目标点，确保局部规划器始终持有有效航点
      publishCurrentGoal();
    }
  }

private:
  void loadParameters() {
    pnh_.param<std::string>("topics/odom_topic", odom_topic_, "/state_estimation");
    pnh_.param<std::string>("topics/point_goal_topic", point_goal_topic_, "/way_point");
    pnh_.param<bool>("topics/publish_point_goal", publish_point_goal_, true);
    pnh_.param<std::string>("topics/pose_goal_topic", pose_goal_topic_, "/move_base_simple/goal");
    pnh_.param<bool>("topics/publish_pose_goal", publish_pose_goal_, true);
    pnh_.param<std::string>("topics/speed_topic", speed_topic_, "/speed");
    pnh_.param<bool>("topics/publish_speed", publish_speed_, true);
    pnh_.param<double>("topics/speed_value", speed_value_, 1.0);
    pnh_.param<std::string>("topics/frame_id", frame_id_, "map");

    pnh_.param<double>("navigation/arrival_radius", arrival_radius_, 0.5);
    pnh_.param<double>("navigation/z_bound", z_bound_, 2.0);
    pnh_.param<bool>("navigation/enable_early_switch", enable_early_switch_, true);
    pnh_.param<double>("navigation/early_switch_radius", early_switch_radius_, 1.2);
    pnh_.param<double>("navigation/wait_time", wait_time_, 0.0);
    pnh_.param<bool>("navigation/loop_navigation", loop_navigation_, false);
    pnh_.param<double>("navigation/control_rate", control_rate_, 20.0);

    pnh_.param<double>("recording/distance_interval", distance_interval_, 1.0);
    pnh_.param<double>("recording/time_interval", time_interval_, 1.0);
    pnh_.param<bool>("recording/record_start_point", record_start_point_, false);

    pnh_.param<std::string>("control/command_topic", command_topic_, "/waypoint_recorder/command");
    pnh_.param<std::string>("control/status_topic", status_topic_, "/waypoint_recorder/status");
    pnh_.param<std::string>("control/marker_topic", marker_topic_, "/waypoint_recorder/markers");
    pnh_.param<std::string>("storage/default_file_path", default_file_path_, "");
  }

  void setupROS() {
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &WaypointNode::odomCallback, this);
    sub_cmd_ = nh_.subscribe<std_msgs::String>(command_topic_, 10, &WaypointNode::commandCallback, this);

    if (publish_point_goal_) {
      pub_point_goal_ = nh_.advertise<geometry_msgs::PointStamped>(point_goal_topic_, 5);
    }
    if (publish_pose_goal_) {
      pub_pose_goal_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_goal_topic_, 5);
    }
    if (publish_speed_) {
      pub_speed_ = nh_.advertise<std_msgs::Float32>(speed_topic_, 5);
    }

    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 5);
    pub_status_ = nh_.advertise<std_msgs::String>(status_topic_, 5);
  }

  void setupDynamicReconfigure() {
    WaypointRecorderConfig config;
    config.odomTopic = odom_topic_;
    config.pointGoalTopic = point_goal_topic_;
    config.publishPointGoal = publish_point_goal_;
    config.poseGoalTopic = pose_goal_topic_;
    config.publishPoseGoal = publish_pose_goal_;
    config.speedTopic = speed_topic_;
    config.publishSpeed = publish_speed_;
    config.speedValue = speed_value_;
    config.frameId = frame_id_;
    config.arrivalRadius = arrival_radius_;
    config.zBound = z_bound_;
    config.enableEarlySwitch = enable_early_switch_;
    config.earlySwitchRadius = early_switch_radius_;
    config.waitTime = wait_time_;
    config.loopNavigation = loop_navigation_;
    config.controlRate = control_rate_;
    config.distanceInterval = distance_interval_;
    config.timeInterval = time_interval_;
    config.recordStartPoint = record_start_point_;
    config.commandTopic = command_topic_;
    config.statusTopic = status_topic_;
    config.markerTopic = marker_topic_;
    config.defaultFilePath = default_file_path_;
    current_config_ = config;

    reconfigure_server_.reset(
        new dynamic_reconfigure::Server<WaypointRecorderConfig>(reconfigure_mutex_, pnh_));
    reconfigure_server_->updateConfig(config);
    reconfigure_server_->setCallback(
        [this](WaypointRecorderConfig& updated_config, uint32_t) {
          reconfigureCallback(updated_config);
        });
  }

  bool validTopic(const std::string& topic) const {
    std::string error;
    return !topic.empty() && ros::names::validate(topic, error);
  }

  void reconfigureCallback(WaypointRecorderConfig& config) {
    if (!validTopic(config.odomTopic) || !validTopic(config.pointGoalTopic) ||
        !validTopic(config.poseGoalTopic) || !validTopic(config.speedTopic) ||
        !validTopic(config.commandTopic) || !validTopic(config.statusTopic) ||
        !validTopic(config.markerTopic)) {
      ROS_ERROR("[WaypointNode] Dynamic reconfigure rejected an empty or invalid topic name.");
      config.odomTopic = odom_topic_;
      config.pointGoalTopic = point_goal_topic_;
      config.poseGoalTopic = pose_goal_topic_;
      config.speedTopic = speed_topic_;
      config.commandTopic = command_topic_;
      config.statusTopic = status_topic_;
      config.markerTopic = marker_topic_;
    }
    if (config.frameId.empty()) {
      ROS_ERROR("[WaypointNode] Dynamic reconfigure rejected an empty frame_id.");
      config.frameId = frame_id_;
    }
    if (config.earlySwitchRadius < config.arrivalRadius) {
      ROS_WARN("[WaypointNode] early_switch_radius is below arrival_radius; raising it to arrival_radius.");
      config.earlySwitchRadius = config.arrivalRadius;
    }

    const bool odom_changed = config.odomTopic != odom_topic_;
    const bool command_changed = config.commandTopic != command_topic_;
    const bool point_goal_changed = config.pointGoalTopic != point_goal_topic_ ||
                                    config.publishPointGoal != publish_point_goal_;
    const bool pose_goal_changed = config.poseGoalTopic != pose_goal_topic_ ||
                                   config.publishPoseGoal != publish_pose_goal_;
    const bool speed_topic_changed = config.speedTopic != speed_topic_;
    const bool speed_changed = speed_topic_changed || config.publishSpeed != publish_speed_ ||
                               config.speedValue != speed_value_;
    const bool marker_changed = config.markerTopic != marker_topic_;
    const bool status_changed = config.statusTopic != status_topic_;
    const bool goal_changed = point_goal_changed || pose_goal_changed || speed_changed ||
                              config.frameId != frame_id_;

    const bool output_disabled = (publish_point_goal_ && !config.publishPointGoal) ||
                                 (publish_pose_goal_ && !config.publishPoseGoal);
    const bool navigation_io_changed = odom_changed ||
        config.pointGoalTopic != point_goal_topic_ || config.poseGoalTopic != pose_goal_topic_ ||
        speed_topic_changed || (publish_speed_ && !config.publishSpeed) || output_disabled;
    if (state_ == NavState::NAVIGATING && navigation_io_changed) {
      publishZeroSpeed();
      state_ = NavState::PAUSED;
      is_waiting_ = false;
      ROS_WARN("[WaypointNode] Navigation paused because live I/O settings changed. Send START to resume.");
    }

    odom_topic_ = config.odomTopic;
    point_goal_topic_ = config.pointGoalTopic;
    publish_point_goal_ = config.publishPointGoal;
    pose_goal_topic_ = config.poseGoalTopic;
    publish_pose_goal_ = config.publishPoseGoal;
    speed_topic_ = config.speedTopic;
    publish_speed_ = config.publishSpeed;
    speed_value_ = config.speedValue;
    frame_id_ = config.frameId;
    arrival_radius_ = config.arrivalRadius;
    z_bound_ = config.zBound;
    enable_early_switch_ = config.enableEarlySwitch;
    early_switch_radius_ = config.earlySwitchRadius;
    wait_time_ = config.waitTime;
    loop_navigation_ = config.loopNavigation;
    control_rate_ = config.controlRate;
    distance_interval_ = config.distanceInterval;
    time_interval_ = config.timeInterval;
    record_start_point_ = config.recordStartPoint;
    command_topic_ = config.commandTopic;
    status_topic_ = config.statusTopic;
    marker_topic_ = config.markerTopic;
    default_file_path_ = config.defaultFilePath;

    if (odom_changed) {
      sub_odom_.shutdown();
      has_odom_ = false;
      sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &WaypointNode::odomCallback, this);
    }
    if (command_changed) {
      sub_cmd_.shutdown();
      sub_cmd_ = nh_.subscribe<std_msgs::String>(command_topic_, 10, &WaypointNode::commandCallback, this);
    }
    if (point_goal_changed) {
      pub_point_goal_.shutdown();
      if (publish_point_goal_) {
        pub_point_goal_ = nh_.advertise<geometry_msgs::PointStamped>(point_goal_topic_, 5);
      }
    }
    if (pose_goal_changed) {
      pub_pose_goal_.shutdown();
      if (publish_pose_goal_) {
        pub_pose_goal_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_goal_topic_, 5);
      }
    }
    if (speed_changed) {
      pub_speed_.shutdown();
      if (publish_speed_) {
        pub_speed_ = nh_.advertise<std_msgs::Float32>(speed_topic_, 5);
      }
    }
    if (marker_changed) {
      pub_markers_.shutdown();
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 5);
    }
    if (status_changed) {
      pub_status_.shutdown();
      pub_status_ = nh_.advertise<std_msgs::String>(status_topic_, 5);
    }
    if (state_ == NavState::NAVIGATING && goal_changed) {
      publishCurrentGoal();
    }
    current_config_ = config;
  }

  void publishZeroSpeed() {
    if (!publish_speed_) return;
    std_msgs::Float32 stop_msg;
    stop_msg.data = 0.0f;
    pub_speed_.publish(stop_msg);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& position = msg->pose.pose.position;
    const double yaw = tf::getYaw(msg->pose.pose.orientation);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(yaw)) {
      ROS_WARN_THROTTLE(2.0, "[WaypointNode] Ignoring non-finite odometry pose.");
      return;
    }
    if (recording_mode_ == RecordingMode::DISTANCE) {
      const double dx = position.x - vehicle_x_;
      const double dy = position.y - vehicle_y_;
      const double dz = position.z - vehicle_z_;
      recording_distance_ += std::hypot(std::hypot(dx, dy), dz);
    }
    has_odom_ = true;
    last_odom_time_ = msg->header.stamp.toSec();
    vehicle_x_ = msg->pose.pose.position.x;
    vehicle_y_ = msg->pose.pose.position.y;
    vehicle_z_ = msg->pose.pose.position.z;
    vehicle_yaw_ = yaw;
    updateRecording();
  }

  void commandCallback(const std_msgs::String::ConstPtr& msg) {
    std::string cmd = msg->data;
    ROS_INFO_STREAM("[WaypointNode] Received command: " << cmd);

    if (cmd == "RECORD") {
      recordCurrentPose();
    } else if (cmd == "RECORD_DISTANCE") {
      startRecording(RecordingMode::DISTANCE);
    } else if (cmd == "RECORD_TIME") {
      startRecording(RecordingMode::TIME);
    } else if (cmd == "STOP_RECORDING") {
      stopRecording();
    } else if (cmd == "START" || cmd == "PLAY") {
      startNavigation();
    } else if (cmd.find("START_FROM:") == 0) {
      std::istringstream index_stream(cmd.substr(11));
      size_t waypoint_number = 0;
      char trailing_character = 0;
      if (!(index_stream >> waypoint_number) || (index_stream >> trailing_character) ||
          waypoint_number == 0) {
        ROS_WARN("[WaypointNode] Invalid START_FROM index. Use a 1-based positive waypoint number.");
        return;
      }
      startNavigationFromIndex(waypoint_number - 1);
    } else if (cmd == "PAUSE") {
      pauseNavigation();
    } else if (cmd == "CONTINUE" || cmd == "RESUME") {
      continueNavigation();
    } else if (cmd == "STOP") {
      stopNavigation();
    } else if (cmd == "CLEAR") {
      clearWaypoints();
    } else if (cmd == "UNDO") {
      undoWaypoint();
    } else if (cmd == "TOGGLE_EARLY_SWITCH") {
      enable_early_switch_ = !enable_early_switch_;
      current_config_.enableEarlySwitch = enable_early_switch_;
      if (reconfigure_server_) {
        reconfigure_server_->updateConfig(current_config_);
      }
      ROS_INFO_STREAM("[WaypointNode] Early switch is now: " << (enable_early_switch_ ? "ENABLED" : "DISABLED"));
    } else if (cmd.find("SAVE_FORMAT:") == 0) {
      saveWaypointsInFormat(cmd.substr(12));
    } else if (cmd.find("SAVE:") == 0) {
      std::string path = cmd.substr(5);
      if (path.empty()) path = default_file_path_;
      manager_.saveToFile(path);
    } else if (cmd.find("LOAD:") == 0) {
      if (recording_mode_ != RecordingMode::OFF) {
        ROS_WARN("[WaypointNode] Stop recording before loading waypoints.");
        return;
      }
      std::string path = cmd.substr(5);
      if (path.empty()) path = default_file_path_;
      if (manager_.loadFromFile(path)) {
        current_idx_ = 0;
        state_ = NavState::IDLE;
      }
    } else {
      ROS_WARN_STREAM("[WaypointNode] Unknown command received: " << cmd);
    }
  }

  enum class RecordingMode { OFF, DISTANCE, TIME };

  const char* recordingModeName() const {
    switch (recording_mode_) {
      case RecordingMode::DISTANCE: return "DISTANCE";
      case RecordingMode::TIME: return "TIME";
      default: return "OFF";
    }
  }

  void startRecording(RecordingMode mode) {
    if (recording_mode_ != RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] Recording already active. Stop it before starting another session.");
      return;
    }
    if (state_ == NavState::NAVIGATING) {
      ROS_WARN("[WaypointNode] Pause navigation before starting automatic recording.");
      return;
    }
    if (!has_odom_) {
      ROS_WARN("[WaypointNode] Cannot start recording: No odometry received yet!");
      return;
    }
    const double interval = mode == RecordingMode::DISTANCE ? distance_interval_ : time_interval_;
    if (!std::isfinite(interval) || interval <= 0.0) {
      ROS_ERROR("[WaypointNode] Cannot start recording: recording/%s_interval must be finite and positive.",
                mode == RecordingMode::DISTANCE ? "distance" : "time");
      return;
    }
    recording_mode_ = mode;
    recording_distance_ = 0.0;
    recording_time_ = ros::Time::now();
    if (record_start_point_) {
      recordCurrentPose();
    }
    ROS_INFO("[WaypointNode] Automatic recording started: %s, interval %.3f %s, record start point: %s.",
             recordingModeName(), interval, mode == RecordingMode::DISTANCE ? "m" : "s",
             record_start_point_ ? "ON" : "OFF");
  }

  void updateRecording() {
    if (recording_mode_ == RecordingMode::OFF) {
      return;
    }
    const ros::Time now = ros::Time::now();
    if (now < recording_time_) {
      ROS_WARN("[WaypointNode] ROS time moved backwards; resetting recording time baseline.");
      recording_time_ = now;
    }
    const bool due = recording_mode_ == RecordingMode::DISTANCE
        ? recording_distance_ >= distance_interval_
        : (now - recording_time_).toSec() >= time_interval_;
    if (due) {
      recordCurrentPose();
      recording_distance_ = 0.0;
      recording_time_ = now;
    }
  }

  void stopRecording() {
    if (recording_mode_ == RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] No automatic recording session to stop.");
      return;
    }
    recordCurrentPose();
    recording_mode_ = RecordingMode::OFF;
    ROS_INFO("[WaypointNode] Automatic recording stopped; final pose recorded. Use SAVE to persist waypoints.");
  }

  void recordCurrentPose() {
    if (!has_odom_) {
      ROS_WARN("[WaypointNode] Cannot record waypoint: No odometry received yet!");
      return;
    }
    double stamp = (last_odom_time_ > 0.0) ? last_odom_time_ : ros::Time::now().toSec();
    manager_.addWaypoint(vehicle_x_, vehicle_y_, vehicle_z_, vehicle_yaw_, stamp);
    size_t count = manager_.size();
    const Waypoint* wp = manager_.getWaypoint(count - 1);
    ROS_INFO("[WaypointNode] >>> Recorded Waypoint #%d: [X=%.2f, Y=%.2f, Z=%.2f, Yaw=%.2frad] (Total: %zu)",
             wp->id, wp->x, wp->y, wp->z, wp->yaw, count);
  }

  void saveWaypointsInFormat(const std::string& format) {
    if (default_file_path_.empty()) {
      ROS_ERROR("[WaypointNode] Cannot save by format: storage/default_file_path is empty.");
      return;
    }

    std::string base_path = default_file_path_;
    const size_t slash = base_path.find_last_of('/');
    const size_t dot = base_path.find_last_of('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
      base_path.erase(dot);
    }

    bool saved = true;
    if (format == "YAML" || format == "ALL") {
      saved = manager_.saveToFile(base_path + ".yaml") && saved;
    }
    if (format == "PLY" || format == "ALL") {
      saved = manager_.saveToFile(base_path + ".ply") && saved;
    }
    if (format != "YAML" && format != "PLY" && format != "ALL") {
      ROS_WARN_STREAM("[WaypointNode] Unknown save format: " << format);
    } else if (!saved) {
      ROS_ERROR_STREAM("[WaypointNode] Failed to save waypoint file(s) for format " << format);
    }
  }

  void startNavigation() {
    if (state_ == NavState::PAUSED) {
      ROS_WARN("[WaypointNode] Navigation is paused. Use CONTINUE to resume or STOP to reset.");
      return;
    }
    const size_t start_idx = state_ == NavState::COMPLETED ? 0 : current_idx_;
    startNavigationFromIndex(start_idx);
  }

  void startNavigationFromIndex(size_t start_idx) {
    if (recording_mode_ != RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] Stop recording before starting navigation.");
      return;
    }
    if (manager_.empty()) {
      ROS_WARN("[WaypointNode] Cannot start navigation: Waypoint list is empty!");
      return;
    }
    if (start_idx >= manager_.size()) {
      ROS_WARN("[WaypointNode] Cannot start navigation: waypoint index %zu is outside [1, %zu].",
               start_idx + 1, manager_.size());
      return;
    }
    if (!has_odom_) {
      ROS_WARN("[WaypointNode] Cannot start navigation: No odometry received yet!");
      return;
    }
    current_idx_ = start_idx;
    state_ = NavState::NAVIGATING;
    is_waiting_ = false;
    ROS_INFO("[WaypointNode] Starting navigation from WP %d / %zu",
             static_cast<int>(current_idx_ + 1), manager_.size());
    publishCurrentGoal();
  }

  void pauseNavigation() {
    if (state_ != NavState::NAVIGATING) {
      ROS_WARN("[WaypointNode] Navigation is not running; nothing to pause.");
      return;
    }
    state_ = NavState::PAUSED;
    is_waiting_ = false;
    publishZeroSpeed();
    ROS_INFO("[WaypointNode] Navigation paused at the current waypoint.");
  }

  void continueNavigation() {
    if (state_ != NavState::PAUSED) {
      ROS_WARN("[WaypointNode] Navigation is not paused; nothing to continue.");
      return;
    }
    if (recording_mode_ != RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] Stop recording before continuing navigation.");
      return;
    }
    if (manager_.empty() || current_idx_ >= manager_.size()) {
      ROS_WARN("[WaypointNode] Cannot continue navigation: current waypoint is unavailable.");
      stopNavigation();
      return;
    }
    if (!has_odom_) {
      ROS_WARN("[WaypointNode] Cannot continue navigation: No odometry received yet!");
      return;
    }
    state_ = NavState::NAVIGATING;
    is_waiting_ = false;
    ROS_INFO("[WaypointNode] Continuing navigation from WP %d / %zu",
             static_cast<int>(current_idx_ + 1), manager_.size());
    publishCurrentGoal();
  }

  void stopNavigation() {
    const bool was_active = state_ == NavState::NAVIGATING || state_ == NavState::PAUSED;
    if (was_active) {
      publishZeroSpeed();
    }
    state_ = NavState::IDLE;
    current_idx_ = 0;
    is_waiting_ = false;
    wait_start_time_ = 0.0;
    ROS_INFO("[WaypointNode] Navigation stopped; next start will begin at WP 1.");
  }

  void clearWaypoints() {
    if (recording_mode_ != RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] Stop recording before clearing waypoints.");
      return;
    }
    state_ = NavState::IDLE;
    current_idx_ = 0;
    is_waiting_ = false;
    manager_.clearWaypoints();
    ROS_INFO("[WaypointNode] All waypoints cleared.");
  }

  void undoWaypoint() {
    if (recording_mode_ != RecordingMode::OFF) {
      ROS_WARN("[WaypointNode] Stop recording before undoing waypoints.");
      return;
    }
    if (manager_.undoLastWaypoint()) {
      if (current_idx_ >= manager_.size() && current_idx_ > 0) {
        current_idx_ = manager_.size() - 1;
      }
      ROS_INFO("[WaypointNode] Last waypoint removed. Remaining count: %zu", manager_.size());
    } else {
      ROS_WARN("[WaypointNode] Waypoint list is already empty, nothing to undo.");
    }
  }

  void publishCurrentGoal() {
    if (current_idx_ >= manager_.size()) {
      return;
    }

    const Waypoint* wp = manager_.getWaypoint(current_idx_);
    if (!wp) return;

    ros::Time now = ros::Time::now();

    // 1. 发布 PointStamped (匹配 local_planner /way_point 接口)
    if (publish_point_goal_) {
      geometry_msgs::PointStamped pt_msg;
      pt_msg.header.stamp = now;
      pt_msg.header.frame_id = frame_id_;
      pt_msg.point.x = wp->x;
      pt_msg.point.y = wp->y;
      pt_msg.point.z = wp->z;
      pub_point_goal_.publish(pt_msg);
    }

    // 2. 发布 PoseStamped (匹配通用 ROS 导航 /move_base_simple/goal 接口)
    if (publish_pose_goal_) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = now;
      pose_msg.header.frame_id = frame_id_;
      pose_msg.pose.position.x = wp->x;
      pose_msg.pose.position.y = wp->y;
      pose_msg.pose.position.z = wp->z;
      pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(wp->yaw);
      pub_pose_goal_.publish(pose_msg);
    }

    // 3. 发布速度指令 (触发小车运动)
    if (publish_speed_) {
      std_msgs::Float32 spd_msg;
      spd_msg.data = static_cast<float>(speed_value_);
      pub_speed_.publish(spd_msg);
    }
  }

  void publishMarkers() {
    if (pub_markers_.getNumSubscribers() == 0) {
      return;
    }

    visualization_msgs::MarkerArray markers;
    ros::Time now = ros::Time::now();

    // 清理旧标记
    visualization_msgs::Marker del_marker;
    del_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(del_marker);

    const auto& wps = manager_.getWaypoints();
    if (wps.empty()) {
      pub_markers_.publish(markers);
      return;
    }

    // 轨迹线 Marker
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = frame_id_;
    path_marker.header.stamp = now;
    path_marker.ns = "waypoint_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.08;
    path_marker.color.r = 0.0f;
    path_marker.color.g = 0.8f;
    path_marker.color.b = 1.0f;
    path_marker.color.a = 0.8f;

    for (const auto& wp : wps) {
      geometry_msgs::Point pt;
      pt.x = wp.x;
      pt.y = wp.y;
      pt.z = wp.z;
      path_marker.points.push_back(pt);
    }
    markers.markers.push_back(path_marker);

    // 每个航点的圆球和编号文本
    for (size_t i = 0; i < wps.size(); ++i) {
      const auto& wp = wps[i];
      bool is_current = (state_ == NavState::NAVIGATING && i == current_idx_);

      // 圆球 Marker
      visualization_msgs::Marker sphere_marker;
      sphere_marker.header.frame_id = frame_id_;
      sphere_marker.header.stamp = now;
      sphere_marker.ns = "waypoint_spheres";
      sphere_marker.id = wp.id;
      sphere_marker.type = visualization_msgs::Marker::SPHERE;
      sphere_marker.action = visualization_msgs::Marker::ADD;
      sphere_marker.pose.position.x = wp.x;
      sphere_marker.pose.position.y = wp.y;
      sphere_marker.pose.position.z = wp.z;
      sphere_marker.pose.orientation.w = 1.0;

      if (is_current) {
        sphere_marker.scale.x = 0.45;
        sphere_marker.scale.y = 0.45;
        sphere_marker.scale.z = 0.45;
        sphere_marker.color.r = 1.0f; // 当前目标高亮为黄色
        sphere_marker.color.g = 0.9f;
        sphere_marker.color.b = 0.0f;
        sphere_marker.color.a = 0.95f;
      } else {
        sphere_marker.scale.x = 0.3;
        sphere_marker.scale.y = 0.3;
        sphere_marker.scale.z = 0.3;
        sphere_marker.color.r = 0.1f; // 其他目标为绿色
        sphere_marker.color.g = 0.9f;
        sphere_marker.color.b = 0.2f;
        sphere_marker.color.a = 0.8f;
      }
      markers.markers.push_back(sphere_marker);

      // 文本编号 Marker
      visualization_msgs::Marker text_marker;
      text_marker.header.frame_id = frame_id_;
      text_marker.header.stamp = now;
      text_marker.ns = "waypoint_labels";
      text_marker.id = wp.id;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.pose.position.x = wp.x;
      text_marker.pose.position.y = wp.y;
      text_marker.pose.position.z = wp.z + 0.35;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.28;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 0.95f;
      text_marker.text = std::to_string(wp.id);
      markers.markers.push_back(text_marker);
    }

    pub_markers_.publish(markers);
  }

  void publishStatus() {
    if (pub_status_.getNumSubscribers() == 0) {
      return;
    }
    std::stringstream ss;
    ss << "STATE:" << navStateToString(state_)
       << "|COUNT:" << manager_.size()
       << "|CURRENT:" << (manager_.empty() ? 0 : current_idx_ + 1)
       << "|EARLY_SWITCH:" << (enable_early_switch_ ? "ON" : "OFF")
       << "|RECORDING:" << recordingModeName()
       << "|ODOM:" << (has_odom_ ? "OK" : "NO");
    std_msgs::String msg;
    msg.data = ss.str();
    pub_status_.publish(msg);
  }

public:
  double getControlRate() const { return control_rate_; }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_cmd_;
  ros::Publisher pub_point_goal_;
  ros::Publisher pub_pose_goal_;
  ros::Publisher pub_speed_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_status_;
  boost::recursive_mutex reconfigure_mutex_;
  std::unique_ptr<dynamic_reconfigure::Server<WaypointRecorderConfig>> reconfigure_server_;
  WaypointRecorderConfig current_config_;

  WaypointManager manager_;
  size_t current_idx_;
  NavState state_;
  bool is_waiting_;
  double wait_start_time_;

  bool has_odom_;
  double vehicle_x_;
  double vehicle_y_;
  double vehicle_z_;
  double vehicle_yaw_;
  double last_odom_time_;

  RecordingMode recording_mode_ = RecordingMode::OFF;
  double recording_distance_ = 0.0;
  ros::Time recording_time_;
  double distance_interval_;
  double time_interval_;
  bool record_start_point_;

  // 参数
  std::string odom_topic_;
  std::string point_goal_topic_;
  bool publish_point_goal_;
  std::string pose_goal_topic_;
  bool publish_pose_goal_;
  std::string speed_topic_;
  bool publish_speed_;
  double speed_value_;
  std::string frame_id_;

  double arrival_radius_;
  double z_bound_;
  bool enable_early_switch_;
  double early_switch_radius_;
  double wait_time_;
  bool loop_navigation_;
  double control_rate_;

  std::string command_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string default_file_path_;
};

} // namespace waypoint_recorder

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  waypoint_recorder::WaypointNode node(nh, pnh);
  double control_rate = node.getControlRate();
  ros::Rate rate(control_rate);

  while (ros::ok()) {
    ros::spinOnce();
    node.update();
    if (node.getControlRate() != control_rate) {
      control_rate = node.getControlRate();
      rate = ros::Rate(control_rate);
    }
    rate.sleep();
  }

  return 0;
}
