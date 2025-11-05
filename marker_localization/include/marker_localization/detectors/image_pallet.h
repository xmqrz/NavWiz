/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tang Swee Ho
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_IMAGE_PALLET_H
#define MARKER_LOCALIZATION_DETECTORS_IMAGE_PALLET_H

#include <image_transport/image_transport.h>
#include <marker_localization/detector.h>

#include <sensor_msgs/Image.h>


namespace marker_localization
{

class ImagePalletDetector: public Detector
{
public:
  ImagePalletDetector(MarkerLocalization& ml, const std::string& sensor) :
    Detector(ml, sensor),
    use_mask(false)
  {}

  void start();
  void stop();

private:
  void handleCameraInfo(const sensor_msgs::CameraInfo& msg);
  void handleDepth(const sensor_msgs::ImageConstPtr& msg);
  void handleImage(const sensor_msgs::ImageConstPtr& msg);
  void matchingMethod(int, void*);

private:
  ros::Subscriber cinfo_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Subscriber image_sub_;

  // variables
  double distance1 = 0.0;
  double distance2 = 0.0;
  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;
  sensor_msgs::CameraInfo cameraInfo;
  bool use_mask;
  int match_method;
  cv::Mat img;
  cv::Mat templ;
  cv::Mat mask;
  cv::Mat result;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_IMAGE_PALLET_H
