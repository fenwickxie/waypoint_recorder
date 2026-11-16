#include <iostream>
#include <cctype>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Config.h>
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
      : nh_(nh), pnh_(pnh), last_status_("WAITING"), awaiting_save_format_(false),
        awaiting_start_index_(false) {
    pnh_.param<std::string>("command_topic", command_topic_, "/waypoint_recorder/command");
    pnh_.param<std::string>("status_topic", status_topic_, "/waypoint_recorder/status");
    pnh_.param<std::string>("default_file_path", default_file_path_, "");
    pnh_.param<std::string>("dynamic_config_topic", dynamic_config_topic_,
                            "/waypoint_node/parameter_updates");

    pub_cmd_ = nh_.advertise<std_msgs::String>(command_topic_, 10);
    sub_status_ = nh_.subscribe<std_msgs::String>(status_topic_, 10, &WaypointTeleop::statusCallback, this);
    sub_dynamic_config_ = nh_.subscribe<dynamic_reconfigure::Config>(
        dynamic_config_topic_, 1, &WaypointTeleop::dynamicConfigCallback, this);

    printHelp();
  }

  void statusCallback(const std_msgs::String::ConstPtr& msg) {
    last_status_ = msg->data;
  }

  void dynamicConfigCallback(const dynamic_reconfigure::Config::ConstPtr& msg) {
    std::string updated_command_topic = command_topic_;
    std::string updated_status_topic = status_topic_;
    for (const auto& parameter : msg->strs) {
      if (parameter.name == "commandTopic") {
        updated_command_topic = parameter.value;
      } else if (parameter.name == "statusTopic") {
        updated_status_topic = parameter.value;
      } else if (parameter.name == "defaultFilePath") {
        default_file_path_ = parameter.value;
      }
    }

    if (updated_command_topic != command_topic_) {
      pub_cmd_.shutdown();
      command_topic_ = updated_command_topic;
      pub_cmd_ = nh_.advertise<std_msgs::String>(command_topic_, 10);
    }
    if (updated_status_topic != status_topic_) {
      sub_status_.shutdown();
      status_topic_ = updated_status_topic;
      sub_status_ = nh_.subscribe<std_msgs::String>(
          status_topic_, 10, &WaypointTeleop::statusCallback, this);
    }
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
    std::cout << "  [d] : 开始按行驶距离自动录制 (Record by distance)\n";
    std::cout << "  [f] : 开始按时间间隔自动录制 (Record by time)\n";
    std::cout << "  [e] : 结束自动录制并补录终点 (End recording)\n";
    std::cout << "  [p] : 开始自动连续发布航点 (Start navigation)\n";
    std::cout << "  [n] : 输入航点序号，从该点开始发布后续航点 (Start from waypoint)\n";
    std::cout << "  [b] : 暂停导航发布 (Pause; resumable)\n";
    std::cout << "  [g] : 从暂停位置继续导航 (Continue / Resume)\n";
    std::cout << "  [s] : 彻底停止导航并从首个航点重新开始 (Stop / Reset)\n";
    std::cout << "  [u] : 撤销上一个航点 (Undo last waypoint)\n";
    std::cout << "  [c] : 清空所有录制的航点 (Clear all waypoints)\n";
    std::cout << "  [t] : 开关提前发布顺滑切换 (Toggle smooth early-switch)\n";
    std::cout << "  [w] : 选择格式保存 (YAML / PLY / 全部)\n";
    std::cout << "  [l] : 从文件读取加载航点序列 (Load from disk)\n";
    std::cout << "  [i] : 打印当前状态信息 (Print status)\n";
    std::cout << "  [h] : 打印此帮助菜单 (Help)\n";
    std::cout << "  [q] : 退出交互控制台 (Quit)\n";
    std::cout << "===============================================================\n";
    std::cout << "默认保存/读取路径: " << (default_file_path_.empty() ? "系统默认" : default_file_path_) << "\n";
    std::cout << "按键即刻生效，无需按回车；序号从 1 开始。输入序号时按回车确认、x 取消。\n";
    std::cout << "保存时按 y/p/a 选择 YAML/PLY/全部，x 取消。\n\n";
  }

  void run() {
    TerminalController term;
    ros::Rate rate(20);

    while (ros::ok()) {
      ros::spinOnce();
      char key = term.readKey();

      if (key != 0) {
        if (awaiting_start_index_) {
          handleStartIndexKey(key);
          rate.sleep();
          continue;
        }

        if (awaiting_save_format_) {
          handleSaveFormatKey(key);
          rate.sleep();
          continue;
        }

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
          case 'n':
          case 'N':
            start_index_input_.clear();
            awaiting_start_index_ = true;
            std::cout << ">> 输入起始航点序号（从 1 开始，回车确认，x 取消）: ";
            std::cout.flush();
            break;
          case 'd':
          case 'D':
            std::cout << ">> [RECORD_DISTANCE] 开始按距离自动录制...\n";
            sendCommand("RECORD_DISTANCE");
            break;
          case 'f':
          case 'F':
            std::cout << ">> [RECORD_TIME] 开始按时间自动录制...\n";
            sendCommand("RECORD_TIME");
            break;
          case 'e':
          case 'E':
            std::cout << ">> [STOP_RECORDING] 结束自动录制并补录终点...\n";
            sendCommand("STOP_RECORDING");
            break;
          case 's':
          case 'S':
            std::cout << ">> [STOP] 彻底停止导航并重置到首个航点...\n";
            sendCommand("STOP");
            break;
          case 'b':
          case 'B':
            std::cout << ">> [PAUSE] 暂停导航发布，可按 g 继续...\n";
            sendCommand("PAUSE");
            break;
          case 'g':
          case 'G':
            std::cout << ">> [CONTINUE] 从暂停位置继续导航...\n";
            sendCommand("CONTINUE");
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
            std::cout << ">> 保存格式 [y] YAML / [p] PLY / [a] 全部 / [x] 取消: ";
            std::cout.flush();
            awaiting_save_format_ = true;
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
  void handleStartIndexKey(char key) {
    if (key == '\r' || key == '\n') {
      awaiting_start_index_ = false;
      if (start_index_input_.empty()) {
        std::cout << "\n未输入序号，已取消。\n";
        return;
      }
      std::cout << "\n>> [START_FROM] 从航点 " << start_index_input_ << " 开始发布...\n";
      sendCommand("START_FROM:" + start_index_input_);
      start_index_input_.clear();
      return;
    }
    if (key == 'x' || key == 'X' || key == 27) {
      awaiting_start_index_ = false;
      start_index_input_.clear();
      std::cout << "\n已取消。\n";
      return;
    }
    if (key == 8 || key == 127) {
      if (!start_index_input_.empty()) {
        start_index_input_.pop_back();
        std::cout << "\b \b";
        std::cout.flush();
      }
      return;
    }
    if (std::isdigit(static_cast<unsigned char>(key))) {
      start_index_input_.push_back(key);
      std::cout << key;
      std::cout.flush();
    }
  }

  void handleSaveFormatKey(char key) {
    awaiting_save_format_ = false;
    switch (key) {
      case 'y':
      case 'Y':
        std::cout << "YAML\n>> [SAVE] 保存航点为 YAML...\n";
        sendCommand("SAVE_FORMAT:YAML");
        break;
      case 'p':
      case 'P':
        std::cout << "PLY\n>> [SAVE] 保存航点为 PLY...\n";
        sendCommand("SAVE_FORMAT:PLY");
        break;
      case 'a':
      case 'A':
        std::cout << "全部格式\n>> [SAVE] 同时保存 YAML 和 PLY...\n";
        sendCommand("SAVE_FORMAT:ALL");
        break;
      case 'x':
      case 'X':
      case 27:
        std::cout << "取消保存。\n";
        break;
      default:
        std::cout << "无效选择，已取消保存。\n";
        break;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_cmd_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_dynamic_config_;

  std::string command_topic_;
  std::string status_topic_;
  std::string dynamic_config_topic_;
  std::string default_file_path_;
  std::string last_status_;
  bool awaiting_save_format_;
  bool awaiting_start_index_;
  std::string start_index_input_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_teleop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  WaypointTeleop teleop(nh, pnh);
  teleop.run();

  return 0;
}
