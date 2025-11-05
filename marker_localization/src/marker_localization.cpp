/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <marker_localization/detector.h>
#include <marker_localization/detectors/ar.h>
#include <marker_localization/detectors/bar_ar.h>
#include <marker_localization/detectors/bar_reflector.h>
#include <marker_localization/detectors/custom.h>
#include <marker_localization/detectors/image_pallet.h>
#include <marker_localization/detectors/pallet.h>
#include <marker_localization/detectors/reflector.h>
#include <marker_localization/detectors/vmarker.h>
#include <marker_localization/marker_localization.h>


namespace marker_localization
{

MarkerLocalization::MarkerLocalization() :
  tfl_(tf_)
{
  // ROS subscribers
  ros::NodeHandle nh;
  marker_type_sub_ = nh.subscribe("/marker_type", 1, &MarkerLocalization::handleMarkerType, this);

  // dynamic reconfigure
  ds_.setCallback(boost::bind(&MarkerLocalization::callbackConfig, this, _1, _2));

  // diagnostic updater
  diagnostic_updater_.setHardwareID("AGV05");
  diagnostic_updater_.add("Status", this, &MarkerLocalization::diagnosticStatus);

  // ROS timer for updating diagnostic
  timer_ = nh.createTimer(ros::Duration(1.0), &MarkerLocalization::timerCallback, this);
}

void MarkerLocalization::handleMarkerType(const std_msgs::String& msg)
{
  if (marker_type_ == msg.data) return;
  marker_type_ = msg.data;

  if (detector_)
  {
    detector_->stop();
    detector_.reset();
  }

  if (marker_type_.empty()) return;

  int index = marker_type_.find("__");
  if (index == std::string::npos)
  {
    ROS_ERROR("Invalid marker_type");
    return;
  }

  std::string method = marker_type_.substr(0, index);
  std::string sensor = marker_type_.substr(index + 2);

  uint8_t profile;
  switch (method.back())
  {
  default:
  case '1': profile = 1; break;
  case '2': profile = 2; break;
  case '3': profile = 3; break;
  case '4': profile = 4; break;
  case '5': profile = 5; break;
  }

  if (method.find("ar") == 0)
  {
    ROS_INFO("Starting AR detector");
    detector_ = boost::make_shared<ARDetector>(boost::ref(*this), sensor, profile);
  }
  else if (method == "image_pallet")
  {
    ROS_INFO("Starting image pallet detector");
    detector_ = boost::make_shared<ImagePalletDetector>(boost::ref(*this), sensor);
  }
  else if (method.find("pallet") == 0)
  {
    ROS_INFO("Starting pallet detector");
    detector_ = boost::make_shared<PalletDetector>(boost::ref(*this), sensor, profile);
  }
  else if (method.find("reflector") == 0)
  {
    ROS_INFO("Starting reflector detector");
    detector_ = boost::make_shared<ReflectorDetector>(boost::ref(*this), sensor, profile);
  }
  else if (method.find("vmarker") == 0)
  {
    ROS_INFO("Starting vmarker detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<VExtraction>());
  }
  else if (method.find("lmarker") == 0)
  {
    ROS_INFO("Starting lmarker detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<LExtraction>());
  }
  else if (method.find("vlmarker") == 0)
  {
    ROS_INFO("Starting vlmarker detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<VLExtraction>());
  }
  else if (method.find("v2marker") == 0)
  {
    ROS_INFO("Starting v2marker detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<V2Extraction>());
  }
  else if (method.find("l2marker") == 0)
  {
    ROS_INFO("Starting l2marker detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<L2Extraction>());
  }
  else if (method.find("barhead") == 0)
  {
    ROS_INFO("Starting barhead detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<BarExtraction>(true));
  }
  else if (method.find("bartail") == 0)
  {
    ROS_INFO("Starting bartail detector");
    detector_ = boost::make_shared<VMarkerDetector>(boost::ref(*this), sensor, profile, boost::make_shared<BarExtraction>(false));
  }
  else if (method.find("barar") == 0)
  {
    ROS_INFO("Starting barar detector");
    detector_ = boost::make_shared<BarARDetector>(boost::ref(*this), sensor, profile);
  }
  else if (method.find("barreflector") == 0)
  {
    ROS_INFO("Starting barreflector detector");
    detector_ = boost::make_shared<BarReflectorDetector>(boost::ref(*this), sensor, profile);
  }
  else if (method.find("custom") == 0)
  {
    ROS_INFO("Starting custom detector");
    detector_ = boost::make_shared<CustomDetector>(boost::ref(*this), sensor, profile);
  }
  else
  {
    ROS_ERROR_STREAM("No detector found for marker_type: " << marker_type_);
  }

  if (detector_) detector_->start();
}

void MarkerLocalization::diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (detector_)
  {
    auto marker = detector_->getMarkerInOdom();
    stat.add("Marker Type", marker_type_);
    if (!std::isnan(marker.x))
    {
      stat.add("Marker X", marker.x);
      stat.add("Marker Y", marker.y);
      stat.add("Marker Theta", marker.theta * 180 / M_PI);
    }
    else
    {
      stat.add("Marker X", "-");
      stat.add("Marker Y", "-");
      stat.add("Marker Theta", "-");
    }
  }
  else
  {
    stat.add("Marker Type", "-");
    stat.add("Marker X", "-");
    stat.add("Marker Y", "-");
    stat.add("Marker Theta", "-");
  }
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}

}  // namespace marker_localization


int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_localization");
  ROS_INFO("marker_localization started.");

  marker_localization::MarkerLocalization ml;
  ros::spin();
}
