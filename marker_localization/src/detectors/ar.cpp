/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tang Swee Ho
 */

#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <marker_localization/detectors/ar.h>
#include <tf2/LinearMath/Transform.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>


#if ARUCO_VERSION_MAJOR < 3
namespace aruco_ros
{

tf2::Transform arucoMarker2Tf2(const aruco::Marker& marker)
{
  tf::Transform t = arucoMarker2Tf(marker);
  tf::Quaternion q = t.getRotation();
  tf::Vector3 v = t.getOrigin();
  return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(v.x(), v.y(), v.z()));
}

}  // namespace aruco_ros
#endif


namespace marker_localization
{

bool ARDetector::start(bool cloud_flatten)
{
  camParam_ = aruco::CameraParameters();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  Detector::start(nh, cloud_flatten, true);
  if (!marker_list_pub_)
  {
    // publishers are static variables and only registered once
    // publisher shutdown is avoided as it takes time for its peers to resubscribe
    marker_list_pub_ = nh.advertise<std_msgs::UInt32MultiArray>("ar/markers_list", 1);
    image_pub_ = it.advertise("ar/result", 1);
    debug_pub_ = it.advertise("ar/debug", 1);
  }

  // determine ID and range
  // possible string formats:
  //  {sensor}__{ID} or {sensor}__{ID_1}_{ID_2} or {sensor}__r{rangeMax} or
  //  {sensor}__{ID}__r{rangeMax} or {sensor}__{ID_1}_{ID_2}__r{rangeMax}
  // input units: range -> meters
  id_[0] = id_[1] = -1;
  trim_range_max_ = std::numeric_limits<float>::max();

  std::string id_str;
  int index = sensor_.find("__");
  if (index != std::string::npos)
  {
    id_str = sensor_.substr(index + 2);
    sensor_ = sensor_.substr(0, index);
  }
  std::string range_str;
  index = id_str.find("__r");
  if (index != std::string::npos)
  {
    range_str = id_str.substr(index + 3);
    id_str = id_str.substr(0, index);
  }
  else if (id_str.front() == 'r')
  {
    range_str = id_str.substr(1);
    id_str.clear();
  }

  if (!id_str.empty())
  {
    index = id_str.find("_");
    if (index == std::string::npos)
    {
      std::istringstream iss(id_str);
      float id;

      if (iss >> id)
      {
        id_[0] = id;
      }
    }
    else
    {
      std::istringstream iss(id_str.substr(0, index));
      std::istringstream iss2(id_str.substr(index + 1));
      float id[2];

      if ((iss >> id[0]) && (iss2 >> id[1]))
      {
        id_[0] = id[0];
        id_[1] = id[1];
      }
    }
  }

  if (!range_str.empty())
  {
    std::istringstream iss(range_str);
    float range;

    if (iss >> range)
    {
      trim_range_max_ = fabs(range);
    }
  }

  std::string topic = getCameraTopic(sensor_.substr(0, sensor_.find('_')));
  if (topic.length())
  {
    // Update camera model
    auto info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic + "/color/camera_info", nh, ros::Duration(2.0));
    if (!info)
    {
      ROS_ERROR_STREAM("No camera info: " << sensor_);
      return false;
    }

    camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*info, *config_ar_.image_is_rectified);
    if (!camParam_.isValid())
    {
      ROS_ERROR_STREAM("Invalid camera info: " << sensor_);
      return false;
    }

    image_sub_ = it.subscribe(topic + "/color/image_raw", 1, &ARDetector::callbackImage, this);
  }
  else
  {
    ROS_ERROR_STREAM("No sensor found: " << sensor_);
    return false;
  }
  return true;
}

void ARDetector::stop()
{
  image_sub_.shutdown();
  Detector::stop();
}

void ARDetector::getMarkers(const sensor_msgs::ImageConstPtr& msg, geometry_msgs::PoseArray& marker_msg)
{
  ROS_DEBUG("Image received");

  cv_bridge::CvImagePtr cv_ptr;

  std::vector<aruco::Marker> markers;
  std_msgs::UInt32MultiArray marker_id_msg;
  cv::Mat inImage;

  std::string docking_frame_id = "laser";  // TODO(someday): hack-ish, create new urdf for stabilized camera link

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

    bool roi = *config_ar_.height_start || *config_ar_.height_end || *config_ar_.width_start || *config_ar_.width_end;
    cv::Mat img, &r_img = *config_ar_.sharp_strength ? roi ? img : inImage : cv_ptr->image;

    // sharpen the image
    if (*config_ar_.sharp_strength)
    {
      cv::GaussianBlur(cv_ptr->image, r_img, cv::Size(0, 0), *config_ar_.sharp_sigma);
      cv::addWeighted(cv_ptr->image, 1.0 + *config_ar_.sharp_strength, r_img, -*config_ar_.sharp_strength, 0, r_img);
    }
    else if (!roi)
    {
      inImage = r_img;
    }

    // region of interest
    if (roi)
    {
      uint32_t x1 = *config_ar_.width_start * msg->width * 0.01;
      uint32_t x2 = msg->width;
      uint32_t y1 = *config_ar_.height_start * msg->height * 0.01;
      uint32_t y2 = msg->height;
      if (*config_ar_.width_end > *config_ar_.width_start)
      {
        x2 *= (*config_ar_.width_end * 0.01);
      }
      x2 -= x1;
      if (*config_ar_.height_end > *config_ar_.height_start)
      {
        y2 *= (*config_ar_.height_end * 0.01);
      }
      y2 -= y1;

      cv::Mat mask(msg->height, msg->width, CV_8UC1, cv::Scalar::all(0));
      mask(cv::Rect(x1, y1, x2, y2)).setTo(cv::Scalar::all(255));
      r_img.copyTo(inImage, mask);
    }

    // ok, let's detect
    mDetector_.detect(inImage, markers, camParam_, *config_ar_.marker_size, false);

    // marker array publish
    marker_msg.header = msg->header;
    marker_msg.poses.clear();
    marker_id_msg.data.clear();

    geometry_msgs::TransformStamped camera_in_base;
    tf2::Stamped<tf2::Transform> camera_in_base_tf;
    try
    {
      camera_in_base = tf_.lookupTransform(docking_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(0.5));
      tf2::fromMsg(camera_in_base, camera_in_base_tf);

      marker_msg.header.frame_id = docking_frame_id;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("Transform Exception");

      camera_in_base_tf.setIdentity();
    }

    for (size_t i = 0; i < markers.size(); ++i)
    {
      if (id_[0] >= 0)
      {
        if (markers[i].id != id_[0] && markers[i].id != id_[1])
        {
          continue;
        }
      }

      tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
      if (transform.getOrigin().length() > trim_range_max_)
      {
        continue;
      }

      geometry_msgs::Pose marker;
      tf2::toMsg(camera_in_base_tf * transform, marker);
      marker_msg.poses.push_back(marker);
      marker_id_msg.data.push_back(markers[i].id);
    }

    // publish marker array
    landmarks_pub_.publish(marker_msg);

    if (marker_list_pub_.getNumSubscribers() > 0)
    {
      marker_list_pub_.publish(marker_id_msg);
    }

    if (image_pub_.getNumSubscribers() > 0)
    {
      // draw detected markers on the image for visualization
      for (size_t i = 0; i < markers.size(); ++i)
      {
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }

      // draw a 3D cube in each marker if there is 3D info
      if (camParam_.isValid())
      {
        for (size_t i = 0; i < markers.size(); ++i)
          aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam_);
      }

      // show input with augmented information
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = inImage;
      image_pub_.publish(out_msg.toImageMsg());
    }

    // show also the internal image resulting from the threshold operation
    if (debug_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage debug_msg;
      debug_msg.header = msg->header;
      debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
      debug_msg.image = mDetector_.getThresholdedImage();
      debug_pub_.publish(debug_msg.toImageMsg());
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void ARDetector::processData(const sensor_msgs::ImageConstPtr& msg)
{
  geometry_msgs::PoseArray markers;
  getMarkers(msg, markers);

  // TODO(someday): hack-ish, create new urdf for stabilized camera link
  std::string docking_frame_id = "base";
  geometry_msgs::Pose2D sensor;
  if (frameTransform(msg->header, geometry_msgs::Pose2D(), sensor, docking_frame_id))
  {
    std_msgs::Header header = msg->header;
    header.frame_id = docking_frame_id;
    docking_frame_id = "marker_sensor";
    publishTransform(header, sensor, docking_frame_id);
    tf_.lookupTransform(header.frame_id, docking_frame_id, header.stamp, ros::Duration(0.5));
  }

  if (id_[0] >= 0 && id_[1] < 0)
  {
    for (auto& marker : markers.poses)
    {
      tf2::Transform transform;
      tf2::fromMsg(marker, transform);
      tf2::Transform rotation(transform.getRotation());
      if (std::abs(rotation(tf2::Vector3(0.0, 0.0, 1.0)).z()) > 0.5)
      {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, M_PI_2);
        rotation.setRotation(q);
      }
      else
      {
        rotation.setRotation(tf2::Quaternion(-0.5, 0.5, 0.5, 0.5));
      }
      tf2::toMsg(transform * rotation, marker);
    }

    std::vector<geometry_msgs::Pose2D> markers_2d;
    if (frameTransform(markers, &markers_2d, docking_frame_id))
    {
      handleMarkers(markers.header, markers_2d, *config_ar_.max_position_deviation);
    }
  }
  else if (frameTransform(markers, NULL, docking_frame_id))
  {
    handleMarkerPairs(markers, *config_ar_.min_separation,
                      *config_ar_.max_separation, *config_ar_.max_position_deviation);
  }
}


/* static variables */
ros::Publisher ARDetector::marker_list_pub_;
image_transport::Publisher ARDetector::image_pub_;
image_transport::Publisher ARDetector::debug_pub_;

}  // namespace marker_localization
