#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <unistd.h>

#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include "waypoint_recorder/waypoint_manager.h"

class RecordingTest : public testing::Test {
protected:
  void SetUp() override {
    commands_ = nh_.advertise<std_msgs::String>(topic_prefix_ + "/command", 10);
    odometry_ = nh_.advertise<nav_msgs::Odometry>(topic_prefix_ + "/odom", 10);
    status_sub_ = nh_.subscribe(topic_prefix_ + "/status", 10, &RecordingTest::statusCallback, this);
    marker_sub_ = nh_.subscribe(topic_prefix_ + "/markers", 10, &RecordingTest::markerCallback, this);
    ASSERT_TRUE(waitFor([&] {
      return commands_.getNumSubscribers() > 0 && odometry_.getNumSubscribers() > 0 &&
             !status_.empty();
    }));
    command("STOP_RECORDING");
    command("CLEAR");
    ASSERT_TRUE(expectCount(0));
  }

  void TearDown() override {
    if (!temporary_file_.empty()) std::remove(temporary_file_.c_str());
  }

  bool waitFor(const std::function<bool()>& condition, double seconds = 2.0) {
    const auto deadline = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (condition()) return true;
      ros::WallDuration(0.005).sleep();
    }
    return false;
  }

  void settle(double seconds = 0.1) {
    waitFor([] { return false; }, seconds);
  }

  void command(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    commands_.publish(message);
    settle();
  }

  void pose(double x, double y = 0.0, double z = 0.0, double yaw = 0.0) {
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time::now();
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = z;
    message.pose.pose.orientation.z = std::sin(yaw / 2.0);
    message.pose.pose.orientation.w = std::cos(yaw / 2.0);
    odometry_.publish(message);
    settle();
  }

  bool expectCount(size_t count) {
    return waitFor([&] {
      return points_.size() == count &&
             status_.find("|COUNT:" + std::to_string(count) + "|") != std::string::npos;
    });
  }

  void statusCallback(const std_msgs::String::ConstPtr& message) {
    status_ = message->data;
  }

  void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& message) {
    points_.clear();
    for (const auto& marker : message->markers) {
      if (marker.ns == "waypoint_path") points_ = marker.points;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher commands_, odometry_;
  ros::Subscriber status_sub_, marker_sub_;
  std::string topic_prefix_ = "/recording_test";
  std::string status_;
  std::string temporary_file_;
  std::vector<geometry_msgs::Point> points_;
};

TEST_F(RecordingTest, RejectsMissingOdometryAndInvalidIntervals) {
  std::string status;
  auto subscriber = nh_.subscribe<std_msgs::String>("/recording_invalid_test/status", 10,
      [&](const std_msgs::String::ConstPtr& message) { status = message->data; });
  auto commands = nh_.advertise<std_msgs::String>("/recording_invalid_test/command", 10);
  auto odometry = nh_.advertise<nav_msgs::Odometry>("/recording_invalid_test/odom", 10);
  ASSERT_TRUE(waitFor([&] {
    return commands.getNumSubscribers() > 0 && odometry.getNumSubscribers() > 0 && !status.empty();
  }));
  const auto send = [&](const std::string& value) {
    std_msgs::String message;
    message.data = value;
    commands.publish(message);
    settle();
    EXPECT_NE(std::string::npos, status.find("|COUNT:0|"));
    EXPECT_NE(std::string::npos, status.find("|RECORDING:OFF"));
  };
  EXPECT_NE(std::string::npos, status.find("|ODOM:NO"));
  send("RECORD_DISTANCE");
  send("RECORD_TIME");
  nav_msgs::Odometry message;
  message.pose.pose.orientation.w = 1.0;
  odometry.publish(message);
  ASSERT_TRUE(waitFor([&] { return status.find("|ODOM:OK") != std::string::npos; }));
  send("RECORD_DISTANCE");
  send("RECORD_TIME");
}

TEST_F(RecordingTest, DistanceAccumulatesTurnsAndHeightAndRecordsEndpoint) {
  pose(0.0);
  command("RECORD_DISTANCE");
  ASSERT_TRUE(expectCount(1));
  pose(0.5);
  EXPECT_TRUE(expectCount(1));
  pose(0.0);
  ASSERT_TRUE(expectCount(2));
  EXPECT_DOUBLE_EQ(0.0, points_.back().x);
  pose(0.0, 0.0, 1.0);
  ASSERT_TRUE(expectCount(3));
  EXPECT_DOUBLE_EQ(1.0, points_.back().z);
  pose(0.2, 0.0, 1.0);
  EXPECT_TRUE(expectCount(3));
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(4));
  EXPECT_DOUBLE_EQ(0.2, points_.back().x);
  EXPECT_DOUBLE_EQ(1.0, points_.back().z);
  pose(5.0);
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(4));
}

TEST_F(RecordingTest, DistanceThresholdAndOvershootRecordOnlyActualSamples) {
  pose(0.0);
  command("RECORD_DISTANCE");
  pose(0.999);
  EXPECT_TRUE(expectCount(1));
  pose(1.0);
  EXPECT_TRUE(expectCount(2));
  pose(4.5);
  EXPECT_TRUE(expectCount(3));
  pose(5.0);
  EXPECT_TRUE(expectCount(3));
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(4));
  EXPECT_DOUBLE_EQ(5.0, points_.back().x);
}

TEST_F(RecordingTest, RepeatedStartsAndEditingDoNotCorruptRecording) {
  pose(0.0);
  command("RECORD_DISTANCE");
  command("RECORD_DISTANCE");
  command("RECORD_TIME");
  command("CLEAR");
  command("UNDO");
  command("LOAD:/nonexistent.yaml");
  command("START");
  EXPECT_TRUE(expectCount(1));
  EXPECT_NE(std::string::npos, status_.find("|RECORDING:DISTANCE"));
  EXPECT_NE(std::string::npos, status_.find("STATE:IDLE"));
  pose(0.25);
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(2));
  pose(10.0);
  command("START");
  EXPECT_NE(std::string::npos, status_.find("STATE:NAVIGATING"));
  command("RECORD_TIME");
  EXPECT_NE(std::string::npos, status_.find("|RECORDING:OFF"));
  command("STOP");
}

TEST_F(RecordingTest, ImmediateStopStillRecordsEndpointAndRestartResetsDistance) {
  pose(0.0);
  command("RECORD_DISTANCE");
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(2));
  command("RECORD_DISTANCE");
  pose(0.5);
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(4));
  command("RECORD_DISTANCE");
  pose(1.0);
  EXPECT_TRUE(expectCount(5));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(6));
}

TEST_F(RecordingTest, TimeRecordsStationarySamplesAndFinalPose) {
  pose(0.0);
  command("RECORD_TIME");
  ASSERT_TRUE(expectCount(1));
  pose(0.0);
  EXPECT_TRUE(expectCount(1));
  settle(0.31);
  EXPECT_TRUE(expectCount(1));
  pose(0.0);
  EXPECT_TRUE(expectCount(2));
  pose(0.1);
  EXPECT_TRUE(expectCount(2));
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(3));
  EXPECT_DOUBLE_EQ(0.1, points_.back().x);
  settle(0.31);
  pose(0.5);
  EXPECT_TRUE(expectCount(3));
}

TEST_F(RecordingTest, ManualRecordAndNavigationStopKeepOriginalMeaning) {
  pose(0.0);
  command("RECORD");
  EXPECT_TRUE(expectCount(1));
  command("RECORD_DISTANCE");
  pose(0.5);
  command("RECORD");
  EXPECT_TRUE(expectCount(3));
  command("STOP");
  EXPECT_NE(std::string::npos, status_.find("|RECORDING:DISTANCE"));
  pose(1.0);
  EXPECT_TRUE(expectCount(4));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(5));
}

TEST_F(RecordingTest, InvalidOdometryDoesNotPolluteDistanceOrEndpoint) {
  pose(0.0);
  command("RECORD_DISTANCE");
  pose(std::numeric_limits<double>::quiet_NaN());
  pose(0.5);
  EXPECT_TRUE(expectCount(1));
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(2));
  EXPECT_DOUBLE_EQ(0.5, points_.back().x);
}

TEST_F(RecordingTest, FinalPosePersistsWithYawAndTimestamp) {
  char path[] = "/tmp/waypoint_recording_test_XXXXXX";
  const int descriptor = mkstemp(path);
  ASSERT_NE(-1, descriptor);
  close(descriptor);
  temporary_file_ = path;

  pose(0.0);
  command("RECORD_DISTANCE");
  pose(0.1, 0.2, 0.3, 0.75);
  command("STOP_RECORDING");
  command("SAVE:" + temporary_file_);

  waypoint_recorder::WaypointManager saved;
  ASSERT_TRUE(saved.loadFromFile(temporary_file_));
  ASSERT_EQ(2u, saved.size());
  const auto* endpoint = saved.getWaypoint(1);
  EXPECT_DOUBLE_EQ(0.1, endpoint->x);
  EXPECT_DOUBLE_EQ(0.2, endpoint->y);
  EXPECT_DOUBLE_EQ(0.3, endpoint->z);
  EXPECT_DOUBLE_EQ(0.75, endpoint->yaw);
  EXPECT_NEAR(ros::Time::now().toSec(), endpoint->stamp, 2.0);
}

class RecordingWithoutStartTest : public RecordingTest {
protected:
  void SetUp() override {
    topic_prefix_ = "/recording_no_start_test";
    RecordingTest::SetUp();
  }
};

TEST_F(RecordingWithoutStartTest, DistanceStartsCountingWithoutRecordingInitialPose) {
  pose(10.0);
  command("RECORD_DISTANCE");
  ASSERT_TRUE(expectCount(0));
  EXPECT_NE(std::string::npos, status_.find("|RECORDING:DISTANCE"));
  pose(10.5);
  EXPECT_TRUE(expectCount(0));
  command("RECORD_DISTANCE");
  command("RECORD_TIME");
  pose(11.0);
  ASSERT_TRUE(expectCount(1));
  EXPECT_DOUBLE_EQ(11.0, points_.front().x);
  pose(11.2);
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(2));
  EXPECT_DOUBLE_EQ(11.2, points_.back().x);
}

TEST_F(RecordingWithoutStartTest, TimeWaitsForIntervalAndStillRecordsEndpoint) {
  pose(0.0);
  command("RECORD_TIME");
  ASSERT_TRUE(expectCount(0));
  pose(0.0);
  EXPECT_TRUE(expectCount(0));
  settle(0.31);
  EXPECT_TRUE(expectCount(0));
  pose(0.1);
  ASSERT_TRUE(expectCount(1));
  EXPECT_DOUBLE_EQ(0.1, points_.front().x);
  pose(0.2);
  command("STOP_RECORDING");
  ASSERT_TRUE(expectCount(2));
  EXPECT_DOUBLE_EQ(0.2, points_.back().x);
}

TEST_F(RecordingWithoutStartTest, ImmediateStopRecordsEndpointInBothModes) {
  pose(0.0);
  command("RECORD_DISTANCE");
  EXPECT_TRUE(expectCount(0));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(1));
  command("RECORD_TIME");
  EXPECT_TRUE(expectCount(1));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(2));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(2));
}

TEST_F(RecordingWithoutStartTest, RestartResetsDistanceAndManualRecordingStillWorks) {
  pose(0.0);
  command("RECORD");
  command("RECORD_DISTANCE");
  pose(0.5);
  EXPECT_TRUE(expectCount(1));
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(2));
  command("RECORD_DISTANCE");
  pose(1.0);
  EXPECT_TRUE(expectCount(2));
  command("RECORD");
  EXPECT_TRUE(expectCount(3));
  pose(1.5);
  ASSERT_TRUE(expectCount(4));
  EXPECT_DOUBLE_EQ(1.5, points_.back().x);
  command("STOP_RECORDING");
  EXPECT_TRUE(expectCount(5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "recording_test");
  return RUN_ALL_TESTS();
}
