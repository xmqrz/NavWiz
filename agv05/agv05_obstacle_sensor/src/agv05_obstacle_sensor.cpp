/*
 * Copyright (c) 2017, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#include <agv05_obstacle_sensor/agv05_obstacle_sensor.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>


// Sensor Group
enum Group
{
  GROUP_A = 0,
  GROUP_B,
  GROUP_C,
  GROUP_LAST
};
const char* g_group_name[GROUP_LAST] =
{
  "Group A",
  "Group B",
  "Group C"
};

// ROS publishers and subscribers
ros::Publisher g_obstacle_sensor_pub;
std::vector<ros::Publisher> g_area_pubs[GROUP_LAST];

ros::Subscriber g_obstacle_sensor_area_sub;
std::vector<ros::Subscriber> g_activation_subs;

// dynamic reconfigure
struct Profile
{
  std::map<uint8_t, uint8_t> area[GROUP_LAST];
}
g_profiles[10];

// diagnostic variable
struct Status
{
  int profile;
  std::string area;
  std::string activation;
  std::string hint;
  std::vector<std::string> n_group;
}
g_status = {0, "Area 0", "No obstacle", "", std::vector<std::string>(GROUP_LAST, "-")};

// variables
agv05_msgs::ObstacleSensorArea g_area;
std::vector<agv05_msgs::ObstacleSensor> g_activations;
std::vector<double> g_timeouts;
std::vector<std::string> g_obstacle_hints;

class NavLidarInspector
{
public:
  void init()
  {
    ros::NodeHandle nh;
    dict topics_dict;
    if (LidarInspector::obtainParam(nh, "nav2d_scan_topics", topics_dict))
    {
      watchdogs.clear();
      watchdogs.reserve(topics_dict.size());
      for (dict::const_iterator it = topics_dict.begin(); it != topics_dict.end(); ++it)
      {
        watchdogs.push_back(Watchdog(it->first, it->second));
        watchdogs.back().init(nh);
      }
    }
  }

  void processActivation(const double& now, agv05_msgs::ObstacleSensor& activation)
  {
    if (activation.malfunction) return;

    for (auto& wd : watchdogs)
    {
      WatchdogActivation result = wd.processActivation(now, LaserArea(), activation, 0);
      if (result.malfunction)
      {
        activation.malfunction = true;
        activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MALFUNCTION;
        activation.hint = result.error_msg;
        break;
      }
    }
  }

private:
  std::vector<Watchdog> watchdogs;
} g_lidar_inspector;


void init()
{
  ros::NodeHandle nh("agv05/obstacle_sensor");
  g_obstacle_sensor_pub = nh.advertise<agv05_msgs::ObstacleSensor>("activation", 1, true);
  g_obstacle_sensor_area_sub = nh.subscribe("area", 1, callbackArea);

  g_lidar_inspector.init();
}

std::string generateEnumSrc(dict* p_lidar)
{
  Json::Value enumSrc;
  Json::FastWriter fastWriter;

  // Obstacle List
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  ros::master::V_TopicInfo::iterator it = topics.begin();
  std::vector<std::pair<std::string, std::string> > obstacle_list;
  for (; it != topics.end(); ++it)
  {
    if (it->datatype != "agv05_msgs/ObstacleSensor" ||
        it->name == "/agv05/obstacle_sensor/activation")
    {
      continue;
    }
    std::vector<std::string> topic;
    boost::algorithm::split(topic, it->name, boost::is_any_of("/"));
    std::vector<std::string>::reverse_iterator topicIt = topic.rbegin();
    if (*topicIt != "activation")
    {
      continue;
    }
    ++topicIt;

    std::string ns = boost::algorithm::replace_last_copy(it->name, "/activation", "");
    std::string name = boost::algorithm::replace_all_copy(*topicIt, "_", " ");
    capitalize(name);

    obstacle_list.push_back(std::make_pair(name, ns));
  }
  std::sort(obstacle_list.begin(), obstacle_list.end());

  // convert to Json::Value
  for (std::vector<std::pair<std::string, std::string> >::iterator it = obstacle_list.begin(),
       it_end = obstacle_list.end(); it != it_end; ++it)
  {
    Json::Value item;
    item.append(it->second);
    item.append(it->first);
    enumSrc["obstacle_list"].append(item);

    if (p_lidar)
    {
      if (LidarInspector::param_prefix_.find(it->first) != LidarInspector::param_prefix_.end())
      {
        (*p_lidar)[it->second] = it->first;
      }
    }
  }

  return fastWriter.write(enumSrc);
}

bool enumServerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.message = generateEnumSrc();
  res.success = true;
  return true;
}

// helper function
std::vector<std::string> getListFromCSV(std::string csv)
{
  std::vector<std::string> list;

  // remove whitespace
  std::string _csv = boost::algorithm::replace_all_copy(csv, " ", "");

  if (!_csv.empty() && _csv != "-")
  {
    boost::algorithm::split(list, _csv, boost::is_any_of(","));
  }

  return list;
}

char _capitalAfterSpace(const char& a, const char& b)
{
  if (b == ' ')
  {
    return toupper(a);
  }
  return a;
}
void capitalize(std::string &s)
{
  s[0] = toupper(s[0]);
  std::transform(s.begin() + 1, s.end(), s.begin(), s.begin() + 1, _capitalAfterSpace);
}

// processes functions
void processObstacleSensor(float frequency)
{
  double now = ros::Time::now().toSec();

  // Merge all activations
  agv05_msgs::ObstacleSensor activation;
  for (int i = 0, n = g_activations.size(); i < n; ++i)
  {
    g_activations[i].malfunction |= (g_timeouts[i] < now);

    activation.far_blocked |= g_activations[i].far_blocked;
    activation.middle_blocked |= g_activations[i].middle_blocked;
    activation.near_blocked |= g_activations[i].near_blocked;
    activation.malfunction |= g_activations[i].malfunction;

    // Set hint
    if (g_activations[i].malfunction ||
        (activation.hint.empty() && g_activations[i].near_blocked))
    {
      activation.hint = g_activations[i].hint.empty() ? g_obstacle_hints[i] : g_activations[i].hint;
    }
  }
  if (g_area.profile && g_area.area != agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL)
  {
    g_lidar_inspector.processActivation(now, activation);
  }

  // Apply a 2-second timeout prior to clearing the malfunction and near blocked status.
  static double last_malfunction = 0;
  static double last_near_blocked = 0;
  static std::string last_activation_hint = "";
  if (activation.malfunction)
  {
    last_malfunction = now;
    last_activation_hint = activation.hint;
  }
  else if (now - last_malfunction < 2.0)
  {
    activation.malfunction = true;
    activation.hint = last_activation_hint;
  }
  else if (activation.near_blocked)
  {
    last_near_blocked = now;
    last_activation_hint = activation.hint;
  }
  else if (now - last_near_blocked < 2.0)
  {
    activation.near_blocked = true;
    activation.hint = last_activation_hint;
  }

  // Summarize
  if (activation.malfunction)
  {
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MALFUNCTION;
    g_status.activation = "Malfunction";
  }
  else if (activation.near_blocked)
  {
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_NEAR_BLOCKED;
    g_status.activation = "Near blocked";
  }
  else if (activation.middle_blocked)
  {
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_MIDDLE_BLOCKED;
    g_status.activation = "Middle blocked";
  }
  else if (activation.far_blocked)
  {
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_FAR_BLOCKED;
    g_status.activation = "Far blocked";
  }
  else
  {
    activation.activation = agv05_msgs::ObstacleSensor::OBSTACLE_NORMAL;
    g_status.activation = "No obstacle";
  }
  g_status.hint = activation.hint;

  g_obstacle_sensor_pub.publish(activation);
}

void processObstacleSensorArea(float frequency)
{
  g_status.profile = g_area.profile;
  int area = g_area.area;
  std::vector<int> group_area(GROUP_LAST, area);

  std::ostringstream oss;
  oss << "Area " << area;
  g_status.area = oss.str();

  if (g_status.profile > 0 && g_status.profile <= 10)
  {
    Profile& profile = g_profiles[g_status.profile - 1];
    switch (area)
    {
    case agv05_msgs::ObstacleSensorArea::AREA_FORWARD:
    case agv05_msgs::ObstacleSensorArea::AREA_FORWARD_FAR:
    case agv05_msgs::ObstacleSensorArea::AREA_FORWARD_MIDDLE:
    case agv05_msgs::ObstacleSensorArea::AREA_FORWARD_NEAR:
    case agv05_msgs::ObstacleSensorArea::AREA_FORWARD_END:
    case agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD:
    case agv05_msgs::ObstacleSensorArea::AREA_REVERSE:
    case agv05_msgs::ObstacleSensorArea::AREA_REVERSE_FAR:
    case agv05_msgs::ObstacleSensorArea::AREA_REVERSE_MIDDLE:
    case agv05_msgs::ObstacleSensorArea::AREA_REVERSE_NEAR:
    case agv05_msgs::ObstacleSensorArea::AREA_REVERSE_END:
    case agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE:
    case agv05_msgs::ObstacleSensorArea::AREA_OMNI:
    case agv05_msgs::ObstacleSensorArea::AREA_OMNI_FAR:
    case agv05_msgs::ObstacleSensorArea::AREA_OMNI_MIDDLE:
    case agv05_msgs::ObstacleSensorArea::AREA_OMNI_NEAR:
    case agv05_msgs::ObstacleSensorArea::AREA_OMNI_END:
    case agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL:
    case agv05_msgs::ObstacleSensorArea::AREA_ROTATE_LEFT:
    case agv05_msgs::ObstacleSensorArea::AREA_ROTATE_RIGHT:
      {
        static const std::map<uint8_t, std::string> _area_status =
        {
          {agv05_msgs::ObstacleSensorArea::AREA_FORWARD, "Forward"},
          {agv05_msgs::ObstacleSensorArea::AREA_FORWARD_FAR, "Forward Far"},
          {agv05_msgs::ObstacleSensorArea::AREA_FORWARD_MIDDLE, "Forward Middle"},
          {agv05_msgs::ObstacleSensorArea::AREA_FORWARD_NEAR, "Forward Near"},
          {agv05_msgs::ObstacleSensorArea::AREA_FORWARD_END, "Forward End"},
          {agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD, "Dynamic Forward"},
          {agv05_msgs::ObstacleSensorArea::AREA_REVERSE, "Reverse"},
          {agv05_msgs::ObstacleSensorArea::AREA_REVERSE_FAR, "Reverse Far"},
          {agv05_msgs::ObstacleSensorArea::AREA_REVERSE_MIDDLE, "Reverse Middle"},
          {agv05_msgs::ObstacleSensorArea::AREA_REVERSE_NEAR, "Reverse Near"},
          {agv05_msgs::ObstacleSensorArea::AREA_REVERSE_END, "Reverse End"},
          {agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE, "Dynamic Reverse"},
          {agv05_msgs::ObstacleSensorArea::AREA_OMNI, "Omni"},
          {agv05_msgs::ObstacleSensorArea::AREA_OMNI_FAR, "Omni Far"},
          {agv05_msgs::ObstacleSensorArea::AREA_OMNI_MIDDLE, "Omni Middle"},
          {agv05_msgs::ObstacleSensorArea::AREA_OMNI_NEAR, "Omni Near"},
          {agv05_msgs::ObstacleSensorArea::AREA_OMNI_END, "Omni End"},
          {agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL, "Manual Control"},
          {agv05_msgs::ObstacleSensorArea::AREA_ROTATE_LEFT, "Rotate Left"},
          {agv05_msgs::ObstacleSensorArea::AREA_ROTATE_RIGHT, "Rotate Right"}
        };
        g_status.area = _area_status.at(area);
      }
      for (size_t i = 0; i < GROUP_LAST; i++)
      {
        group_area[i] = profile.area[i][area];
      }
      break;
    default:
      break;
    }
  }

  std_msgs::UInt8 msg;
  for (size_t i = 0; i < GROUP_LAST; i++)
  {
    msg.data = group_area[i];
    for (auto& pub : g_area_pubs[i])
    {
      pub.publish(msg);
    }
  }
}

// callbacks
void callbackArea(const agv05_msgs::ObstacleSensorArea& area)
{
  g_area = area;
}
void callbackActivation(int index, const agv05_msgs::ObstacleSensorConstPtr& activation)
{
  g_activations[index] = *activation;
  g_timeouts[index] = ros::Time::now().toSec() + HEARTBEAT_TIMEOUT;
}

// diagnostic function
void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Obstacle Sensor Profile", g_status.profile);
  stat.add("Obstacle Sensor Area", g_status.area);
  stat.add("Obstacle Sensor Activation", g_status.activation);
  stat.add("Obstacle Sensor Triggered Hint", g_status.hint.empty() ? "-" : g_status.hint);
  for (size_t i = 0; i < GROUP_LAST; i++)
  {
    stat.add(g_group_name[i], g_status.n_group[i]);
  }
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

// callback config
void callbackConfig(agv05_obstacle_sensor::ObstacleSensorConfig& config, uint32_t level)
{
  ROS_INFO_STREAM("agv05_obstacle_sensor: config received");

  g_activation_subs.clear();
  g_activations.clear();
  g_timeouts.clear();
  g_obstacle_hints.clear();
  for (size_t i = 0; i < GROUP_LAST; i++)
  {
    g_area_pubs[i].clear();
  }

  std::string* config_obstacle_sensors[GROUP_LAST] =
  {
    &(config.group_a_obstacle_sensors),
    &(config.group_b_obstacle_sensors),
    &(config.group_c_obstacle_sensors)
  };
  std::vector<std::string> ns_group[GROUP_LAST];
  std::vector<std::string> ns_group_tmp[GROUP_LAST];

  for (size_t i = 0; i < GROUP_LAST; i++)
  {
    ns_group_tmp[i] = getListFromCSV(*(config_obstacle_sensors[i]));
    std::vector<std::string>& ns_bin = i ? ns_group_tmp[0] : ns_group[0];
    for (auto& s : ns_group_tmp[i])
    {
      // remove if it already exists
      if (std::find(ns_bin.begin(), ns_bin.end(), s) == ns_bin.end())
      {
        ns_bin.push_back(s);
        if (i)
        {
          ns_group[i].push_back(s);
        }
      }
    }
    if (ns_group[i].size() != ns_group_tmp[i].size())
    {
      *(config_obstacle_sensors[i]) = boost::algorithm::join(ns_group[i], ",");
      if (i == 0)
      {
        ns_group_tmp[i] = ns_group[i];
      }
    }
  }

  g_activations.resize(ns_group_tmp[0].size());
  g_timeouts.resize(g_activations.size());

  ros::NodeHandle nh;
  dict lidar_dict;
  generateEnumSrc(&lidar_dict);
  for (size_t i = 0; i < GROUP_LAST; i++)
  {
    std::set<std::string> sensors;
    for (const auto& ns : ns_group[i])
    {
      g_activation_subs.push_back(nh.subscribe<agv05_msgs::ObstacleSensor>(ns + "/activation", 1,
                                  boost::bind(&callbackActivation, g_activation_subs.size(), _1)));
      g_area_pubs[i].push_back(nh.advertise<std_msgs::UInt8>(ns + "/area", 1, true));

      std::string name = ns.substr(ns.rfind("/") + 1);
      boost::algorithm::replace_all(name, "_", " ");
      capitalize(name);
      g_obstacle_hints.push_back(name);

      if (lidar_dict[ns].length())
      {
        dict topics_dict;
        LidarInspector::obtainParam(nh, LidarInspector::param_prefix_.at(lidar_dict[ns]), topics_dict);
        for (const auto& topic : topics_dict)
        {
          sensors.insert(topic.second);
        }
      }
      else
      {
        sensors.insert(name);
      }
    }
    g_status.n_group[i] = sensors.size() ? boost::algorithm::join(sensors, " | ") : "-";
  }

#define LOAD_PROFILE_GROUP(i, group, GROUP) \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_FORWARD] = config.group##_forward_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_FORWARD_FAR] = config.group##_forward_far_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_FORWARD_MIDDLE] = config.group##_forward_middle_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_FORWARD_NEAR] = config.group##_forward_near_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_FORWARD_END] = config.group##_forward_end_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_FORWARD] = config.group##_dynamic_forward_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_REVERSE] = config.group##_reverse_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_REVERSE_FAR] = config.group##_reverse_far_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_REVERSE_MIDDLE] = config.group##_reverse_middle_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_REVERSE_NEAR] = config.group##_reverse_near_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_REVERSE_END] = config.group##_reverse_end_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_DYNAMIC_REVERSE] = config.group##_dynamic_reverse_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_OMNI] = config.group##_omni_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_OMNI_FAR] = config.group##_omni_far_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_OMNI_MIDDLE] = config.group##_omni_middle_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_OMNI_NEAR] = config.group##_omni_near_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_OMNI_END] = config.group##_omni_end_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_MANUAL_CONTROL] = config.group##_manual_control_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_ROTATE_LEFT] = config.group##_rotate_left_area_##i; \
  g_profiles[(i)-1].area[GROUP][agv05_msgs::ObstacleSensorArea::AREA_ROTATE_RIGHT] = config.group##_rotate_right_area_##i; \

#define LOAD_PROFILE(i) \
  LOAD_PROFILE_GROUP(i, group_a, GROUP_A); \
  LOAD_PROFILE_GROUP(i, group_b, GROUP_B); \
  LOAD_PROFILE_GROUP(i, group_c, GROUP_C); \

  LOAD_PROFILE(1);
  LOAD_PROFILE(2);
  LOAD_PROFILE(3);
  LOAD_PROFILE(4);
  LOAD_PROFILE(5);
  LOAD_PROFILE(6);
  LOAD_PROFILE(7);
  LOAD_PROFILE(8);
  LOAD_PROFILE(9);
  LOAD_PROFILE(10);

#undef LOAD_PROFILE
#undef LOAD_PROFILE_GROUP
}

// main function
int main(int argc, char** argv)
{
  // initialize ros node
  ros::init(argc, argv, "agv05_obstacle_sensor");
  ROS_INFO_STREAM("agv05_obstacle_sensor start");

  // init
  init();

  // service
  ros::NodeHandle nh("~");
  ros::ServiceServer enumServer = nh.advertiseService("enum_src", enumServerCallback);

  // diagnostic updater
  double expected_frequency = LOOP_FREQUENCY;
  diagnostic_updater::FrequencyStatus diagnostic_frequency(
    diagnostic_updater::FrequencyStatusParam(
      &expected_frequency, &expected_frequency, 0.1, 1),
    "Process Frequency");
  diagnostic_updater::Updater updater;
  updater.setHardwareID("AGV05");
  updater.add(diagnostic_frequency);
  updater.add("Status", diagnosticStatus);

  // dynamic reconfigure
  dynamic_reconfigure::Server<agv05_obstacle_sensor::ObstacleSensorConfig> server;
  server.setCallback(callbackConfig);

  // ros loop
  ros::Rate rate(LOOP_FREQUENCY);
  while (ros::ok())
  {
    ros::spinOnce();
    processObstacleSensor(LOOP_FREQUENCY);
    diagnostic_frequency.tick();
    updater.update();
    processObstacleSensorArea(LOOP_FREQUENCY);
    rate.sleep();
  }
}
