/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: nikfaisal
 */

#ifndef AGV05_MENU_PANEL_AGV05_MENU_PANEL_H
#define AGV05_MENU_PANEL_AGV05_MENU_PANEL_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <dynamic_reconfigure/server.h>
#include <json/json.h>
#include <ros/ros.h>

#include <agv05_executive_msgs/SetModuleRunning.h>
#include <agv05_menu_panel/MenuPanelConfig.h>
#include <agv05_msgs/BatteryState.h>
#include <agv05_msgs/NavActionAction.h>
#include <linux_wifi/WifiStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>


namespace agv05
{

class Agv05MenuPanel
{
public:
  Agv05MenuPanel();
  void process();

private:
  void processTaskRunner();
  void processLocalTask();
  void processLocalTaskSubmenu();
  void processManualControl();
  void processHotspot();

  // callback functions
  void callbackButtonStart(const std_msgs::Bool& msg)
  {
    static bool last_state = false;
    if (last_state && !msg.data) button_start_ = true;  // button released
    last_state = msg.data;
  }
  void callbackButtonStop(const std_msgs::Bool& msg)
  {
    static bool last_state = false;
    if (last_state && !msg.data) button_stop_ = true;  // button released
    last_state = msg.data;
  }
  void callbackButtonMode(const std_msgs::Bool& msg)
  {
    static bool last_state = false;
    if (last_state && !msg.data) button_mode_ = true;  // button released
    last_state = msg.data;
    if (!last_state) last_button_mode_no_press_ = ros::Time::now();
  }
  void callbackModule(const std_msgs::String& msg);
  void callbackModuleManager(const std::string& command, const Json::Value& data);
  void callbackTaskRunner(const std::string& command, const Json::Value& data);
  void callbackBatteryState(const agv05_msgs::BatteryState& msg)
  {
    battery_state_ = msg;
  }
  void callbackWifiStatus(const linux_wifi::WifiStatus& msg)
  {
    wifi_ = msg;
  }
  void callbackNavFeedback(const agv05_msgs::NavActionActionFeedback& msg)
  {
    feedback_ = msg.feedback;
    last_feedback_ = ros::Time::now();
  }
  void callbackShutdown(const std_msgs::Bool& msg)
  {
    shutdown_ = msg.data;
  }
  void callbackConfig(agv05_menu_panel::MenuPanelConfig& config, uint32_t level)
  {
    ROS_INFO("agv05_menu_panel: config received");
    config_ = config;
  }
  void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // helper functions
  void updateLcd()
  {
    lcd_line1_pub_.publish(lcd_line1_);
    lcd_line2_pub_.publish(lcd_line2_);
  }
  void enableLed(bool start, bool stop, bool mode)
  {
    std_msgs::Bool msg;
    msg.data = start;
    led_start_pub_.publish(msg);
    msg.data = stop;
    led_stop_pub_.publish(msg);
    msg.data = mode;
    led_mode_pub_.publish(msg);
  }
  std::string getBatteryWifiStatus()
  {
    // battery
    std::ostringstream oss;
    oss << "Battery";
    if (battery_state_.state == agv05_msgs::BatteryState::AUTO_CHARGING ||
        battery_state_.state == agv05_msgs::BatteryState::MANUAL_CHARGING)
    {
      oss << " charging";
    }
    oss << " @ " << static_cast<int>(5 * round(battery_state_.percentage / 5)) << "%";

    // wifi
    oss << " | Wifi " << wifi_.state;
    if (wifi_.state == "connected")
    {
      oss << " @ ";
      if (wifi_.ip == "") oss << "-";
      else oss << wifi_.ip;
    }
    return oss.str();
  }
  std::string getSafetyStatus()
  {
    safety_triggered_ = false;

    if ((ros::Time::now() - last_feedback_).toSec() > 0.5)
    {
      return "Idle";
    }

    switch (feedback_.status)
    {
    case agv05_msgs::NavActionFeedback::STATUS_NORMAL:
      return "Normal";
    case agv05_msgs::NavActionFeedback::STATUS_PAUSED:
      return "Paused";
    case agv05_msgs::NavActionFeedback::STATUS_NAVIGATION_FAILED:
      return "Navigation failed";
    case agv05_msgs::NavActionFeedback::STATUS_OUT_OF_LINE:
      return "Out of line";
    case agv05_msgs::NavActionFeedback::STATUS_BUMPER_BLOCKED:
      return "Bumper blocked";
    case agv05_msgs::NavActionFeedback::STATUS_EXTERNAL_SAFETY_TRIGGER:
      return "External safety trigger";
    case agv05_msgs::NavActionFeedback::STATUS_EMERGENCY_BUTTON_PRESSED:
      return "Emergency button pressed";
    case agv05_msgs::NavActionFeedback::STATUS_CHARGER_CONNECTED:
      return "Manual charger connected";
    case agv05_msgs::NavActionFeedback::STATUS_LASER_MALFUNCTION:
      return "Laser malfunction";
    case agv05_msgs::NavActionFeedback::STATUS_LINE_SENSOR_ERROR:
      return "Line sensor error";
    case agv05_msgs::NavActionFeedback::STATUS_DRIVE_OVERLIMIT_ERROR:
      return "Drive overlimit error";
    case agv05_msgs::NavActionFeedback::STATUS_MOTOR_FAULT:
      return "Motor fault";
    case agv05_msgs::NavActionFeedback::STATUS_WHEEL_SLIPPAGE:
      return "Wheel slippage";
    case agv05_msgs::NavActionFeedback::STATUS_OBSTACLE_BLOCKED:
      return "Obstacle blocked";
    case agv05_msgs::NavActionFeedback::STATUS_OBSTACLE_IN_RANGE:
      return "Obstacle in range";
    case agv05_msgs::NavActionFeedback::STATUS_PLAN_EMPTY:
      return "Path blocked";
    case agv05_msgs::NavActionFeedback::STATUS_SYSTEM_ERROR:
      return "Time jumped";
    case agv05_msgs::NavActionFeedback::STATUS_WAIT_TRAFFIC:
      return "Waiting for traffic controller";
    case agv05_msgs::NavActionFeedback::STATUS_SAFETY_TRIGGERED:
      safety_triggered_ = true;
      return "Safety triggered. Press start button to continue.";
    }
    return "Idle";
  }
  void enableHotspot(bool enable)
  {
    std_srvs::SetBool srv;
    srv.request.data = enable;
    wifi_hotspot_srv_.call(srv);
  }
  void setModuleRunning(bool running, const std::string& module_id)
  {
    agv05_executive_msgs::SetModuleRunning srv;
    srv.request.operation = running;
    srv.request.module_id = module_id;
    set_module_running_srv_.call(srv);
  }
  void sendModuleManagerCommand(const std::string& command)
  {
    Json::Value v;
    v["id"] = "__";
    v["data"]["command"] = command;
    writeModulePipe(v);
  }
  void sendTaskRunnerCommand(const std::string& command, const std::string& type = "")
  {
    Json::Value v;
    v["id"] = "task-runner";
    v["data"]["command"] = command;
    v["data"]["type"] = type;  // pre_init_id (for pre_init only)
    writeModulePipe(v);
  }
  void taskRunnerStarted()
  {
    sendTaskRunnerCommand("init_status");
    sendTaskRunnerCommand("list_templates");
    // sendTaskRunnerCommand("list_tasks");
    sendTaskRunnerCommand("get_default_init");
    sendTaskRunnerCommand("is_paused");
  }
  void writeModulePipe(const Json::Value& value)
  {
    Json::FastWriter writer;
    module_.data = writer.write(value);
    module_pub_.publish(module_);
  }

private:
  /* ROS publishers */
  ros::Publisher lcd_line1_pub_;
  ros::Publisher lcd_line2_pub_;
  ros::Publisher led_start_pub_;
  ros::Publisher led_stop_pub_;
  ros::Publisher led_mode_pub_;

  ros::Publisher module_pub_;
  ros::Publisher shutdown_pub_;

  /* ROS subscribers */
  ros::Subscriber button_start_sub_;
  ros::Subscriber button_stop_sub_;
  ros::Subscriber button_mode_sub_;

  ros::Subscriber module_sub_;
  ros::Subscriber battery_state_sub_;
  ros::Subscriber wifi_status_sub_;
  ros::Subscriber nav_feedback_sub_;
  ros::Subscriber navx_feedback_sub_;
  ros::Subscriber shutdown_sub_;

  /* ROS service client */
  ros::ServiceClient set_module_running_srv_;
  ros::ServiceClient wifi_hotspot_srv_;

  /* Dynamic reconfigure server */
  dynamic_reconfigure::Server<agv05_menu_panel::MenuPanelConfig> ds_;

  /* ROS diagnostic */
  double expected_process_frequency_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::FrequencyStatus diagnostic_frequency_;

  /* Data */
  agv05_menu_panel::MenuPanelConfig config_;
  std_msgs::String lcd_line1_;
  std_msgs::String lcd_line2_;
  std_msgs::String module_;

  bool button_start_;
  bool button_stop_;
  bool button_mode_;
  ros::Time last_button_mode_no_press_;

  std::string active_module_;
  std::string init_status_;
  std::vector<std::pair<std::string, std::string> > pre_init_options_;
  bool default_init_;
  int default_init_countdown_;
  bool paused_;
  bool resuming_;
  bool battery_low_;

  agv05_msgs::BatteryState battery_state_;
  linux_wifi::WifiStatus wifi_;
  agv05_msgs::NavActionFeedback feedback_;
  ros::Time last_feedback_;
  bool safety_triggered_;
  bool shutdown_;

  enum Menu
  {
    TASK_RUNNER = 0,
    LOCAL_TASK,
    MANUAL_CONTROL,
    HOTSPOT,
    MENU_N,
    LOCAL_TASK_SUBMENU,
  };
  uint8_t menu_;

  // diagnostic variable
  std::string diagnostic_menu_;
};

}  // namespace agv05

#endif  // AGV05_MENU_PANEL_AGV05_MENU_PANEL_H
