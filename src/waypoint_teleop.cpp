#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class TerminalController {
public:
  TerminalController() {
    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios raw = orig_termios_;
    // 禁用标准模式和回显，进入原始输入模式
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;  // 非阻塞模式，读0字符即可返回
    raw.c_cc[VTIME] = 1; // 100ms 超时
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  }

  ~TerminalController() {
    // 恢复原有终端设置
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
  }

  char readKey() {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) > 0) {
      return c;
    }
    return 0;
  }

private:
  struct termios orig_termios_;
};

class WaypointTeleop {
public:
  WaypointTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh), last_status_("WAITING") {
    pnh_.param<std::string>("command_topic", command_topic_, "/waypoint_recorder/command");
    pnh_.param<std::string>("status_topic", status_topic_, "/waypoint_recorder/status");
    pnh_.param<std::string>("default_file_path", default_file_path_, "");

    pub_cmd_ = nh_.advertise<std_msgs::String>(command_topic_, 10);
    sub_status_ = nh_.subscribe<std_msgs::String>(status_topic_, 10, &WaypointTeleop::statusCallback, this);

    printHelp();
  }

  void statusCallback(const std_msgs::String::ConstPtr& msg) {
    last_status_ = msg->data;
  }

  void sendCommand(const std::string& cmd) {
    std_msgs::String msg;
    msg.data = cmd;
    pub_cmd_.publish(msg);
  }

  void printHelp() {
    std::cout << "\n===============================================================\n";
    std::cout << "          WAYPOINT RECORDER & AUTO-PUBLISHER TELEOP            \n";
    std::cout << "===============================================================\n";
    std::cout << "  [r] : 录制当前位置作为航点 (Record current waypoint)\n";
    std::cout << "  [p] : 开始自动连续发布航点 (Play / Start navigation)\n";
    std::cout << "  [s] : 暂停/停止当前自动发布 (Stop / Pause navigation)\n";
    std::cout << "  [u] : 撤销上一个航点 (Undo last waypoint)\n";
    std::cout << "  [c] : 清空所有录制的航点 (Clear all waypoints)\n";
    std::cout << "  [t] : 开关提前发布顺滑切换 (Toggle smooth early-switch)\n";
    std::cout << "  [w] : 保存航点序列到文件 (Write to disk)\n";
    std::cout << "  [l] : 从文件读取加载航点序列 (Load from disk)\n";
    std::cout << "  [i] : 打印当前状态信息 (Print status)\n";
    std::cout << "  [h] : 打印此帮助菜单 (Help)\n";
    std::cout << "  [q] : 退出交互控制台 (Quit)\n";
    std::cout << "===============================================================\n";
    std::cout << "默认保存/读取路径: " << (default_file_path_.empty() ? "系统默认" : default_file_path_) << "\n";
    std::cout << "按键即刻生效，无需按回车。\n\n";
  }

  void run() {
    TerminalController term;
    ros::Rate rate(20);

    while (ros::ok()) {
      ros::spinOnce();
      char key = term.readKey();

      if (key != 0) {
        switch (key) {
          case 'r':
          case 'R':
            std::cout << ">> [RECORD] 请求录制当前航点...\n";
            sendCommand("RECORD");
            break;
          case 'p':
          case 'P':
            std::cout << ">> [START] 开始自动连续发布目标点...\n";
            sendCommand("START");
            break;
          case 's':
          case 'S':
            std::cout << ">> [STOP] 暂停/停止航点发布...\n";
            sendCommand("STOP");
            break;
          case 'u':
          case 'U':
            std::cout << ">> [UNDO] 撤销上一个录制航点...\n";
            sendCommand("UNDO");
            break;
          case 'c':
          case 'C':
            std::cout << ">> [CLEAR] 清空所有航点...\n";
            sendCommand("CLEAR");
            break;
          case 't':
          case 'T':
            std::cout << ">> [TOGGLE] 切换提前发布顺滑模式开关...\n";
            sendCommand("TOGGLE_EARLY_SWITCH");
            break;
          case 'w':
          case 'W':
            std::cout << ">> [SAVE] 保存航点到文件: " << default_file_path_ << "\n";
            sendCommand("SAVE:" + default_file_path_);
            break;
          case 'l':
          case 'L':
            std::cout << ">> [LOAD] 从文件加载航点: " << default_file_path_ << "\n";
            sendCommand("LOAD:" + default_file_path_);
            break;
          case 'i':
          case 'I':
            std::cout << ">> 当前系统状态: " << last_status_ << "\n";
            break;
          case 'h':
          case 'H':
            printHelp();
            break;
          case 'q':
          case 'Q':
          case 3: // Ctrl+C
            std::cout << ">> 退出 Waypoint Teleop 控制台。\n";
            return;
          default:
            break;
        }
      }
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_cmd_;
  ros::Subscriber sub_status_;

  std::string command_topic_;
  std::string status_topic_;
  std::string default_file_path_;
  std::string last_status_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WaypointTeleop teleop(nh, pnh);
  teleop.run();

  return 0;
}
