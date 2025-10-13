/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: nikfaisal
 */

#include <agv05_menu_panel/agv05_menu_panel.h>

#define LOOP_FREQUENCY 5


namespace agv05
{

Agv05MenuPanel::Agv05MenuPanel() :
  expected_process_frequency_(LOOP_FREQUENCY),
  diagnostic_frequency_(diagnostic_updater::FrequencyStatusParam(&expected_process_frequency_,
                        &expected_process_frequency_), "Process Frequency"),
  button_start_(false),
  button_stop_(false),
  button_mode_(false),
  default_init_(false),
  default_init_countdown_(0),
  paused_(false),
  resuming_(false),
  battery_low_(false),
  safety_triggered_(false),
  shutdown_(false),
  menu_(TASK_RUNNER)
{
  ros::NodeHandle nh;

  // ROS publishers
  lcd_line1_pub_ = nh.advertise<std_msgs::String>("agv05/lcd/line1", 1, true);
  lcd_line2_pub_ = nh.advertise<std_msgs::String>("agv05/lcd/line2", 1, true);
  led_start_pub_ = nh.advertise<std_msgs::Bool>("agv05/panel/control/led_start", 1, true);
  led_stop_pub_ = nh.advertise<std_msgs::Bool>("agv05/panel/control/led_stop", 1, true);
  led_mode_pub_ = nh.advertise<std_msgs::Bool>("agv05/panel/control/led_mode", 1, true);

  module_pub_ = nh.advertise<std_msgs::String>("agv05_executor/module_in", 10, true);
  shutdown_pub_ = nh.advertise<std_msgs::Bool>("shutdown", 1, true);

  // ROS subscribers
  button_start_sub_ = nh.subscribe("agv05/panel/control/button_start", 1,
                                   &Agv05MenuPanel::callbackButtonStart, this);
  button_stop_sub_ = nh.subscribe("agv05/panel/control/button_stop", 1,
                                  &Agv05MenuPanel::callbackButtonStop, this);
  button_mode_sub_ = nh.subscribe("agv05/panel/control/button_mode", 1,
                                  &Agv05MenuPanel::callbackButtonMode, this);

  module_sub_ = nh.subscribe("agv05_executor/module_out", 10,
                             &Agv05MenuPanel::callbackModule, this);
  battery_state_sub_ = nh.subscribe("agv05/power/battery_state", 1,
                                    &Agv05MenuPanel::callbackBatteryState, this);
  wifi_status_sub_ = nh.subscribe("wifi_manager/status", 1,
                                  &Agv05MenuPanel::callbackWifiStatus, this);
  nav_feedback_sub_ = nh.subscribe("nav_action/feedback", 1,
                                   &Agv05MenuPanel::callbackNavFeedback, this);
  navx_feedback_sub_ = nh.subscribe("navx_action/feedback", 1,
                                    &Agv05MenuPanel::callbackNavFeedback, this);
  shutdown_sub_ = nh.subscribe("shutdown", 1, &Agv05MenuPanel::callbackShutdown, this);

  // ROS service clients
  set_module_running_srv_ = nh.serviceClient<agv05_executive_msgs::SetModuleRunning>(
                              "agv05_executor/set_module_running");
  wifi_hotspot_srv_ = nh.serviceClient<std_srvs::SetBool>("wifi_manager/adhoc");

  // dynamic reconfigure
  ds_.setCallback(boost::bind(&Agv05MenuPanel::callbackConfig, this, _1, _2));

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");
  diagnostic_updater_.add(diagnostic_frequency_);
  diagnostic_updater_.add("Status", this, &Agv05MenuPanel::diagnosticStatus);

  // initialize
  lcd_line1_.data = "DF Automation and Robotics";
  lcd_line2_.data = "Initializing...";
  updateLcd();
  sendModuleManagerCommand("active_module");
}

void Agv05MenuPanel::process()
{
  if (shutdown_)
  {
    lcd_line1_.data = "Shutdown sequence initiated";
    lcd_line2_.data = "Powering off AGV";
    enableLed(false, false, false);
    diagnostic_menu_ = "Shutdown";
    updateLcd();
  }
  else if (active_module_ == "map-creator")
  {
    lcd_line1_.data = "Map Creator Mode";
    lcd_line2_.data = getBatteryWifiStatus();
    enableLed(false, false, false);
    diagnostic_menu_ = "Map Creator Mode";
    updateLcd();
  }
  else if (active_module_ == "task-runner" && default_init_)
  {
    std::ostringstream oss;
    oss << "Task Runner (" << std::max(0, default_init_countdown_) << "s)";
    lcd_line1_.data = oss.str();
    lcd_line2_.data = "Press mode to interrupt.";
    enableLed(false, false, true);
    diagnostic_menu_ = "Task Runner Countdown";
    updateLcd();

    if (button_mode_)
    {
      sendTaskRunnerCommand("cancel_default_init");
    }
  }
  else
  {
    if (menu_ == TASK_RUNNER && active_module_ == "task-runner")
    {
      if (button_mode_ && (ros::Time::now() - last_button_mode_no_press_).toSec() > config_.mode_hold_exit_task_runner_timeout)
      {
        menu_ += 1;
      }
    }
    else if (button_mode_ && menu_ != LOCAL_TASK_SUBMENU)
    {
      menu_ += 1;
      if (menu_ >= MENU_N)
      {
        menu_ = 0;
      }
    }

    switch (menu_)
    {
    case TASK_RUNNER:
      processTaskRunner();
      break;
    case LOCAL_TASK:
      processLocalTask();
      break;
    case LOCAL_TASK_SUBMENU:
      processLocalTaskSubmenu();
      break;
    case MANUAL_CONTROL:
      processManualControl();
      break;
    case HOTSPOT:
      processHotspot();
      break;
    }
  }

  // clear button signals
  button_start_ = false;
  button_stop_ = false;
  button_mode_ = false;

  // diagnostic update
  diagnostic_frequency_.tick();
  diagnostic_updater_.update();
}

void Agv05MenuPanel::processTaskRunner()
{
  if (active_module_ == "task-runner")
  {
    std::ostringstream oss;
    oss << "Task Runner Active | Status: " << getSafetyStatus();
    lcd_line1_.data = oss.str();
    lcd_line2_.data = getBatteryWifiStatus();
    enableLed(false, paused_ || safety_triggered_, true);
    diagnostic_menu_ = "Task Runner Active";
    updateLcd();
  }
  else
  {
    lcd_line1_.data = "Task Runner Inactive";
    lcd_line2_.data = "Press start button to enter task runner mode";
    enableLed(true, false, true);
    diagnostic_menu_ = "Task Runner Inactive";

    if (button_start_)
    {
      setModuleRunning(true, "task-runner");
    }
  }
}

void Agv05MenuPanel::processLocalTask()
{
  lcd_line1_.data = "Local Task Mode";
  lcd_line2_.data = " Press start button to view list of task";
  setModuleRunning(false, "task-runner");
  setModuleRunning(false, "manual-control");
  enableLed(true, false, true);
  diagnostic_menu_ = "Local Task";
  updateLcd();

  if (button_start_)
  {
    if (active_module_ != "task-runner")
    {
      setModuleRunning(true, "task-runner");
      sendTaskRunnerCommand("cancel_default_init");
      menu_ = LOCAL_TASK_SUBMENU;
    }
  }
}

void Agv05MenuPanel::processLocalTaskSubmenu()
{
  enableLed(true, true, true);
  diagnostic_menu_ = "Local Task Submenu";

  static int i = 0;
  if (button_mode_) i += 1;
  if (i >= pre_init_options_.size()) i = 0;

  if (pre_init_options_.size() == 0)
  {
    lcd_line2_.data = "No task available.";
  }
  else
  {
    std::ostringstream oss;
    oss << (i + 1) << " " << pre_init_options_[i].first;
    lcd_line2_.data = oss.str();

    if (button_start_)
    {
      sendTaskRunnerCommand("pre_init", pre_init_options_[i].second);
    }
    else if (button_stop_)
    {
      menu_ = LOCAL_TASK;
    }
  }
  updateLcd();
}

void Agv05MenuPanel::processManualControl()
{
  lcd_line1_.data = "Manual Control Mode";
  if (active_module_ == "manual-control")
  {
    lcd_line2_.data = "    Active";
    enableLed(false, true, true);
    diagnostic_menu_ = "Manual Control Active";
  }
  else
  {
    lcd_line2_.data = "    Inactive";
    enableLed(true, false, true);
    diagnostic_menu_ = "Manual Control Inactive";
  }
  updateLcd();

  if (button_start_)
  {
    setModuleRunning(false, "task-runner");
    setModuleRunning(true, "manual-control");
  }
  else if (button_stop_)
  {
    setModuleRunning(false, "manual-control");
  }
}

void Agv05MenuPanel::processHotspot()
{
  lcd_line1_.data = "Hotspot Mode";
  if (wifi_.adhocstate == "on")
  {
    lcd_line2_.data = std::string(" Active @ ") + wifi_.ip;
    enableLed(false, true, true);
    diagnostic_menu_ = "Hotspot Active";
  }
  else
  {
    lcd_line2_.data = "    Inactive";
    enableLed(true, false, true);
    diagnostic_menu_ = "Hotspot Inactive";
  }
  updateLcd();

  if (button_start_)
  {
    enableHotspot(true);
  }
  else if (button_stop_)
  {
    enableHotspot(false);
  }
}

void Agv05MenuPanel::callbackModule(const std_msgs::String& msg)
{
  Json::Reader reader;
  Json::Value value;
  if (!reader.parse(msg.data, value, false) ||
      !value.isObject() ||
      !value["id"].isString() ||
      !value["data"].isObject() ||
      !value["data"]["command"].isString())
  {
    return;
  }

  std::string id = value["id"].asString();
  Json::Value& data = value["data"];
  std::string command = data["command"].asString();
  if (id == "__")
  {
    callbackModuleManager(command, data);
  }
  else if (id == "task-runner")
  {
    callbackTaskRunner(command, data);
  }
}

void Agv05MenuPanel::callbackModuleManager(const std::string& command, const Json::Value& data)
{
  if (command == "active_module")
  {
    active_module_ = data["module_id"].asString();
    if (active_module_ == "task-runner")
    {
      taskRunnerStarted();
    }
  }
  // else if (command == "default_module")
}

void Agv05MenuPanel::callbackTaskRunner(const std::string& command, const Json::Value& data)
{
  if (command == "init_status")
  {
    const Json::Value& status = data["status"];
    if (status.isString())
    {
      init_status_ = status.asString();
    }
  }
  else if (command == "list_templates")
  {
    const Json::Value& pre_init = data["pre_init"];
    if (pre_init.isArray())
    {
      pre_init_options_.clear();
      for (int i = 0, n = pre_init.size(); i < n; ++i)
      {
        if (pre_init[i].isObject())
        {
          const Json::Value& label = pre_init[i]["label"];
          const Json::Value& type = pre_init[i]["type"];
          if (label.isString() && type.isString())
          {
            pre_init_options_.push_back(std::make_pair(label.asString(), type.asString()));
          }
        }
      }
    }
  }
  else if (command == "default_init")
  {
    const Json::Value& default_init = data["default_init"];
    if (default_init.isObject())
    {
      const Json::Value& type = default_init["type"];
      const Json::Value& countdown = default_init["countdown"];
      if (type.isString() && countdown.isInt())
      {
        default_init_ = true;
        default_init_countdown_ = countdown.asInt();
      }
    }
    else
    {
      default_init_ = false;
      default_init_countdown_ = 0;
    }
  }
  else if (command == "is_paused")
  {
    paused_ = data["paused"].asBool();
    resuming_ = data["resuming"].asBool();
    battery_low_ = data["battery_low"].asBool();
  }
}

void Agv05MenuPanel::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("Mode Menu", diagnostic_menu_);
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

}  // namespace agv05

/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "agv05_menu_panel");
  ROS_INFO_STREAM("agv05_menu_panel start");

  agv05::Agv05MenuPanel menu_panel;

  ros::Rate r(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    menu_panel.process();
    r.sleep();
  }
}
