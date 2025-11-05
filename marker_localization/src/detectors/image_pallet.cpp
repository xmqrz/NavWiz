/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tang Swee Ho
 */

#include <cv_bridge/cv_bridge.h>
#include <marker_localization/detectors/image_pallet.h>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef CV_LOAD_IMAGE_COLOR
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#endif


namespace marker_localization
{

void ImagePalletDetector::start()
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  std::string topic = getCameraTopic(sensor_);
  if (topic.length())
  {
    cinfo_sub_ = nh.subscribe(topic + "/color/camera_info", 1, &ImagePalletDetector::handleCameraInfo, this);
    depth_sub_ = it.subscribe(topic + "/depth/image_rect_raw", 1, &ImagePalletDetector::handleDepth, this);
    image_sub_ = it.subscribe(topic + "/aligned_depth_to_color/image_raw", 1, &ImagePalletDetector::handleImage, this);
  }
  else
  {
    ROS_ERROR_STREAM("No sensor found: " << sensor_);
  }
}

void ImagePalletDetector::stop()
{
  cinfo_sub_.shutdown();
  depth_sub_.shutdown();
  image_sub_.shutdown();
}

void ImagePalletDetector::handleCameraInfo(const sensor_msgs::CameraInfo& msg)
{
  ROS_DEBUG("handleCameraInfo");
  cameraInfo = msg;
  ROS_DEBUG("Model: %s", cameraInfo.distortion_model.c_str());
}

void ImagePalletDetector::handleDepth(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("handleDepth");

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  distance1 = 0.001 * cv_ptr->image.at<u_int16_t>(left_y, left_x);
  distance2 = 0.001 * cv_ptr->image.at<u_int16_t>(right_y, right_x);
  ROS_INFO("distance1: %f, distance2: %f", distance1, distance2);

  if (fabs(distance1 - distance2) > 0.3)
  {
    ROS_DEBUG("distance to edge of pallet more than 0.3");
    return;
  }

  // calculate co-ordinate of pallet to realsense
  float x1 = ((left_x - cameraInfo.K.at(2)) / cameraInfo.K.at(0)) * distance1;
  float y1 = ((left_y - cameraInfo.K.at(5)) / cameraInfo.K.at(4)) * distance1;
  float p1_x = x1;
  float p1_y = sqrt(pow(distance1, 2) - pow(x1, 2));

  float x2 = ((right_x - cameraInfo.K.at(2)) / cameraInfo.K.at(0)) * distance2;
  float y2 = ((right_y - cameraInfo.K.at(5)) / cameraInfo.K.at(4)) * distance2;
  float p2_x = x2;
  float p2_y = sqrt(pow(distance2, 2) - pow(x2, 2));

  ROS_DEBUG("p1_x: %f, p1_y: %f, p2_x: %f, p2_y: %f", p1_x, p1_y, p2_x, p2_y);

  typedef std::complex<double> complex;
  complex p1(p1_y, p1_x);
  complex p2(p2_y, p2_x);

  complex c = (p1 + p2) * complex(0.5, 0);
  complex v = p2 - p1;

  complex vp = complex(-v.imag(), v.real());
  if (dot(vp, c) > 0)
  {
    vp = complex(v.imag(), -v.real());
  }

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = msg->header.frame_id;
  transform.header.stamp = msg->header.stamp;
  transform.child_frame_id = "marker";
  transform.transform.translation.x = c.real();
  transform.transform.translation.y = c.imag();

  tf2::Quaternion q;
  q.setRPY(0, 0, std::arg(vp));
  transform.transform.rotation = tf2::toMsg(q);

  try
  {
    transform = tf_.transform(transform, "odom");
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Odom error: " << ex.what());
    return;
  }

  // transform.header.frame_id = "odom";
  // transform.child_frame_id = "marker";
  // tfb_.sendTransform(transform);

  geometry_msgs::Pose2D marker_in_odom;
  marker_in_odom.x = transform.transform.translation.x;
  marker_in_odom.y = transform.transform.translation.y;
  marker_in_odom.theta = tf2::getYaw(transform.transform.rotation);
  handleMarkers(msg->header, {marker_in_odom}, 0.0);
}

void ImagePalletDetector::handleImage(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_DEBUG("handleImage");

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  img = cv_ptr->image;

  templ = cv::imread(config_.template_path, CV_LOAD_IMAGE_COLOR);
  if (use_mask == true)
  {
    mask = cv::imread(config_.mask_path, CV_LOAD_IMAGE_COLOR);
  }

  match_method = 4;
  if (img.empty() || templ.empty() || (use_mask && mask.empty()))
  {
    ROS_INFO("Can't read one of the images");
  }

  matchingMethod(match_method, 0);
  // cv::waitKey(30);
}

void ImagePalletDetector::matchingMethod(int, void*)
{
  cv::Mat img_display;
  img.copyTo(img_display);

  int result_cols = img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create(result_rows, result_cols, CV_32FC1);

  bool method_accepts_mask = (match_method == CV_TM_SQDIFF || match_method == CV_TM_CCORR_NORMED);

#if OPENCV_VERSION_MAJOR >= 3
  if (use_mask && method_accepts_mask)
  {
    cv::matchTemplate(img, templ, result, match_method, mask);
  }
  else
  {
    cv::matchTemplate(img, templ, result, match_method);
  }
#else
  cv::matchTemplate(img, templ, result, match_method);
#endif

  cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  double minVal, maxVal;
  cv::Point minLoc, maxLoc, matchLoc;

  cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

  if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
  {
    matchLoc = minLoc;
  }
  else
  {
    matchLoc = maxLoc;
  }

  if (maxVal > 0.9)
  {
    cv::rectangle(img_display, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar(0, 255, 0), 4);
    cv::rectangle(result, matchLoc, cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), cv::Scalar(0, 255, 0), 4);

    left_x = matchLoc.x;
    left_y = matchLoc.y;
    right_x = matchLoc.x + templ.cols / 2;
    right_y = matchLoc.y;
    ROS_INFO("left_x: %.0f, left_y: %.0f, rigth_x: %.0f, right_y: %.0f", left_x, left_y, right_x, right_y);
  }
}


}  // namespace marker_localization
