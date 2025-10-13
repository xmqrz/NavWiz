/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <agv05_msgs/AudioControl.h>
#include <agv05_panel/PanelConfig.h>
#include <std_msgs/Bool.h>


/* Constants */
const char* BUTTONS[] =
{
  "start",
  "stop",
  "mode",
  "unbrake",
  "power",
};

const char* LEDS[] =
{
  "start",
  "stop",
  "mode",
  "unbrake",
  "low_batt",
  "power",
};

const int BUTTON_COUNT = sizeof(BUTTONS) / sizeof(BUTTONS[0]);
const int POWER_BUTTON = BUTTON_COUNT - 1;

const int LED_COUNT = sizeof(LEDS) / sizeof(LEDS[0]);
const int POWER_LED = LED_COUNT - 1;


/* Globals */
ros::Publisher g_beep_pub;
ros::Publisher g_blink_tick_pub;
ros::Publisher g_button_pubs[BUTTON_COUNT];
ros::Publisher g_led_pubs[LED_COUNT];
ros::Publisher g_shutdown_pub;

std::vector<ros::Subscriber> g_button_subs[BUTTON_COUNT];
ros::Subscriber g_led_subs[LED_COUNT];

ros::Timer g_timer;

agv05_panel::PanelConfig g_config;

agv05_msgs::AudioControl g_beep;
std_msgs::Bool g_blink_tick;
std::vector<bool> g_buttons[BUTTON_COUNT];
std_msgs::Bool g_button_states[BUTTON_COUNT];
bool g_leds[LED_COUNT] = {false};
std_msgs::Bool g_led_states[LED_COUNT];


void callbackButton(int idx, int idx2, const std_msgs::BoolConstPtr& msg)
{
  std::vector<bool>& buttons = g_buttons[idx];
  buttons[idx2] = msg->data;

  bool state = false;
  for (int i = 0, n = buttons.size(); i < n; ++i)
  {
    if (buttons[i])
    {
      state = true;
      break;
    }
  }

  if (g_button_states[idx].data != state)
  {
    g_button_states[idx].data = state;
    g_button_pubs[idx].publish(g_button_states[idx]);
  }
  if (state)
  {
    g_beep_pub.publish(g_beep);
  }
}

void callbackLed(int idx, const std_msgs::BoolConstPtr& msg)
{
  g_leds[idx] = msg->data;
}

void timerCallback(const ros::TimerEvent& event)
{
  g_blink_tick.data = !g_blink_tick.data;
  g_blink_tick_pub.publish(g_blink_tick);

  // Toggle LEDs
  for (int i = 0; i < LED_COUNT - 1; ++i)  // exclude power LED
  {
    bool state = g_leds[i] && g_blink_tick.data;
    if (g_led_states[i].data != state)
    {
      g_led_states[i].data = state;
      g_led_pubs[i].publish(g_led_states[i]);
    }
  }

  // Process power button for shutdown
  static ros::Time shutdown_press;
  static bool shutdown = false;

  if (g_button_states[POWER_BUTTON].data && g_config.power_button_shutdown_timeout)
  {
    if (!shutdown)
    {
      shutdown = true;
      shutdown_press = event.current_real;
    }
    else if ((event.current_real - shutdown_press).toSec() > g_config.power_button_shutdown_timeout)
    {
      g_button_states[POWER_BUTTON].data = false;
      shutdown = false;

      std_msgs::Bool msg;
      msg.data = true;
      g_shutdown_pub.publish(msg);

      ros::Duration(3.0).sleep();
      system("sudo poweroff");
    }
  }
  else
  {
    shutdown = false;
  }
}

void callbackConfig(agv05_panel::PanelConfig& config, uint32_t level)
{
  ROS_INFO("agv05_panel: config received");
  g_config = config;
  g_timer.setPeriod(ros::Duration(1.0 / config.led_blink_frequency / 2));
}

void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.add("Start Button", static_cast<bool>(g_button_states[0].data));
  stat.add("Stop Button", static_cast<bool>(g_button_states[1].data));
  stat.add("Mode Button", static_cast<bool>(g_button_states[2].data));
  stat.add("Unbrake Button", static_cast<bool>(g_button_states[3].data));
  stat.add("Power Button", static_cast<bool>(g_button_states[4].data));

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

void init()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  g_beep_pub = nh.advertise<agv05_msgs::AudioControl>("agv05/audio/beep_control", 1);
  g_beep.operation = agv05_msgs::AudioControl::PLAY;
  g_blink_tick_pub = nh.advertise<std_msgs::Bool>("agv05/led/blink_tick", 1);

  // buttons
  for (int i = 0; i < BUTTON_COUNT; ++i)
  {
    g_button_pubs[i] = nh.advertise<std_msgs::Bool>(
                         std::string("agv05/panel/control/button_") + BUTTONS[i], 2);

    // list of button input topics separated by whitespace
    std::string topics = std::string("agv05/panel/button_") + BUTTONS[i];
    private_nh.getParam(std::string("button_") + BUTTONS[i] + std::string("_topics"), topics);

    std::istringstream iss(topics);
    std::string topic;
    while (iss >> topic)
    {
      ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(topic, 1,
                            boost::bind(&callbackButton, i, g_button_subs[i].size(), _1));
      g_button_subs[i].push_back(sub);
    }
    g_buttons[i].resize(g_button_subs[i].size());

    // initial publish
    g_button_pubs[i].publish(g_button_states[i]);
  }

  // leds
  g_led_states[POWER_LED].data = true;  // turn on power LED

  for (int i = 0; i < LED_COUNT; ++i)
  {
    g_led_pubs[i] = nh.advertise<std_msgs::Bool>(
                      std::string("agv05/panel/led_") + LEDS[i], 1, true);

    g_led_subs[i] = nh.subscribe<std_msgs::Bool>(
                      std::string("agv05/panel/control/led_") + LEDS[i], 1,
                      boost::bind(&callbackLed, i, _1));

    // initial publish
    g_led_pubs[i].publish(g_led_states[i]);
  }

  // shutdown trigger
  g_shutdown_pub = nh.advertise<std_msgs::Bool>("shutdown", 1, true);

  // timer for led blinking
  g_timer = nh.createTimer(ros::Duration(0.5), &timerCallback);
}

/* Main function */
int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "agv05_panel");
  ROS_INFO("agv05_panel start");

  init();

  // diagnostic updater
  diagnostic_updater::Updater updater;
  updater.setHardwareID("AGV05");
  updater.add("Status", diagnosticStatus);
  ros::Timer diagnostic_timer = ros::NodeHandle().createTimer(ros::Duration(1.0),
                                boost::bind(&diagnostic_updater::Updater::update, &updater));

  // dynamic reconfigure
  dynamic_reconfigure::Server<agv05_panel::PanelConfig> ds;
  ds.setCallback(&callbackConfig);

  ros::spin();
}
