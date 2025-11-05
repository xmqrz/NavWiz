/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef MARKER_LOCALIZATION_DETECTOR_H
#define MARKER_LOCALIZATION_DETECTOR_H

#include <marker_localization/marker_localization.h>
#include <marker_localization/utils.h>


namespace marker_localization
{

class Detector
{
public:
  Detector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile = 1) :
    tf_(ml.tf_), tfb_(ml.tfb_), cloud_flatten_pub_(ml.cloud_flatten_pub_), landmarks_pub_(ml.landmarks_pub_),
    config_(ml.config_), sensor_(sensor), profile_(profile), has_first_target_(false)
  {
    marker_in_odom_.x = std::numeric_limits<double>::quiet_NaN();
  }
  virtual ~Detector() {}

  virtual void start() = 0;
  virtual void start(ros::NodeHandle& nh, bool cloud_flatten, bool landmarks)
  {
    // publishers are parent variables and only registered once
    // publisher shutdown is avoided as it takes time for its peers to resubscribe
    if (cloud_flatten && !cloud_flatten_pub_)
    {
      cloud_flatten_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_flatten", 1);
    }
    if (landmarks && !landmarks_pub_)
    {
      // TODO(someday): rename the topic
      landmarks_pub_ = nh.advertise<geometry_msgs::PoseArray>("/docking_reflectors", 1);
    }
  }

  virtual void stop()
  {
    if (cloud_flatten_pub_)
    {
      cloud_flatten_pub_.publish(sensor_msgs::PointCloud2());
    }
    if (landmarks_pub_)
    {
      landmarks_pub_.publish(geometry_msgs::PoseArray());
    }
  }

  virtual geometry_msgs::Pose2D getMarkerInOdom()
  {
    return marker_in_odom_;
  }

protected:  // utility functions
  virtual bool frameTransform(const std_msgs::Header& marker_header,
                              const geometry_msgs::Pose2D& marker,
                              geometry_msgs::Pose2D& marker_new,
                              const std::string& frame_id = "odom")
  {
    ROS_DEBUG_STREAM("marker: " << marker.x  << " " << marker.y << " " << marker.theta);

    if (marker_header.frame_id == frame_id)
    {
      marker_new = marker;
      return true;
    }

    geometry_msgs::TransformStamped transform;

    transform.header.frame_id = marker_header.frame_id;
    transform.header.stamp = ros::Time(0);  // marker_header.stamp;
    transform.child_frame_id = "marker";

    transform.transform.translation.x = marker.x;
    transform.transform.translation.y = marker.y;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, marker.theta);
    transform.transform.rotation = tf2::toMsg(q);

    // Transform to new frame
    try
    {
      transform = tf_.transform(transform, frame_id);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM_THROTTLE(5, frame_id << " transform error: " << ex.what());
      return false;
    }

    marker_new.x = transform.transform.translation.x;
    marker_new.y = transform.transform.translation.y;
    marker_new.theta = tf2::getYaw(transform.transform.rotation);

    return true;
  }

  virtual bool frameTransform(geometry_msgs::PoseArray& markers,
                              std::vector<geometry_msgs::Pose2D>* markers_2d = NULL,
                              const std::string& frame_id = "base")
  {
    if (markers.header.frame_id != frame_id)
    {
      try
      {
        tf_.transform(markers, markers, frame_id);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM_THROTTLE(5, frame_id << " transform error: " << ex.what());
        return false;
      }
    }

    if (markers_2d)
    {
      markers_2d->resize(markers.poses.size());
      for (size_t i = 0; i < markers.poses.size(); ++i)
      {
        geometry_msgs::Pose2D& marker_new = (*markers_2d)[i];
        tf2::Quaternion q;
        tf2::fromMsg(markers.poses[i].orientation, q);
        marker_new.x = markers.poses[i].position.x;
        marker_new.y = markers.poses[i].position.y;
        marker_new.theta = tf2::getYaw(q);
      }
    }

    return true;
  }

  virtual void publishCloudFlatten(const sensor_msgs::LaserScan& scan, const std::set<size_t>& indices,
                                   const std::string& frame_id = "base")
  {
    if (indices.size())
    {
      sensor_msgs::PointCloud2 cloud;
      geometry_msgs::TransformStamped transform;

      transform.header.frame_id = scan.header.frame_id;
      transform.header.stamp = ros::Time(0);  // scan.header.stamp;
      transform.child_frame_id = scan.header.frame_id;

      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      // Transform to new frame
      try
      {
        transform = tf_.transform(transform, frame_id);
        tf2::doTransform(scan, cloud, transform, indices);
        cloud_flatten_pub_.publish(cloud);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM_THROTTLE(5, frame_id << " transform error: " << ex.what());
      }
    }
    else
    {
      cloud_flatten_pub_.publish(sensor_msgs::PointCloud2());
    }
  }

  virtual void publishTransform(const std_msgs::Header& header,
                                const geometry_msgs::Pose2D& marker,
                                const std::string& frame_id)
  {
    geometry_msgs::TransformStamped transform;

    transform.header = header;
    transform.child_frame_id = frame_id;

    transform.transform.translation.x = marker.x;
    transform.transform.translation.y = marker.y;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, marker.theta);
    transform.transform.rotation = tf2::toMsg(q);

    tfb_.sendTransform(transform);
  }

  virtual void publishTransform(const std_msgs::Header& marker_header,
                                const geometry_msgs::Pose2D& marker_in_odom)
  {
    ROS_DEBUG_STREAM("publishing transform");

    geometry_msgs::Pose2D marker;
    if (std::isnan(marker_in_odom_.x))
    {
      marker = marker_in_odom;
      marker.theta = angles::normalize_angle_positive(marker.theta);
      if (marker.theta > (M_PI + M_PI_2))
      {
        marker.theta -= (M_PI * 2);
      }

      size_t mean_filter_window = std::max(1, config_.mean_filter_window);
      marker_x_ = MeanFilter<double>(mean_filter_window, marker.x);
      marker_y_ = MeanFilter<double>(mean_filter_window, marker.y);
      marker_theta_ = MeanFilter<double>(mean_filter_window, marker.theta);
    }
    else
    {
      marker.x = marker_x_.update(marker_in_odom.x);
      marker.y = marker_y_.update(marker_in_odom.y);
      marker.theta = marker_theta_.update(angles::normalize_angle(marker_in_odom.theta
                                          - marker_in_odom_.theta) + marker_in_odom_.theta);
    }

    std_msgs::Header header = marker_header;
    header.frame_id = "odom";
    publishTransform(header, marker, "marker");

    marker_in_odom_ = marker;
  }

  virtual geometry_msgs::Pose2D handleMarkers(const std_msgs::Header& marker_header,
                                              const std::vector<geometry_msgs::Pose2D>& markers,
                                              double max_position_deviation)
  {
    geometry_msgs::Pose2D ret;
    ret.x = std::numeric_limits<double>::quiet_NaN();

    if (markers.size() < 1) return ret;

    double dist = -1.0;
    geometry_msgs::Pose2D marker_in_odom;

    for (const auto& marker : markers)
    {
      double d;
      geometry_msgs::Pose2D marker_new;

      if (!has_first_target_)
      {
        d = std::abs(std::complex<double>(marker.x, marker.y));
      }

      if (!frameTransform(marker_header, marker, marker_new)) continue;

      if (has_first_target_)
      {
        d = poseDistance(marker_new, marker_in_odom_);
        if (max_position_deviation > 0.0 && max_position_deviation < d) continue;
        if (config_.position_deviation_max > 0.0)
        {
          if (config_.position_deviation_max < poseDistance(marker_new, first_in_odom_)) continue;
        }
        if (config_.orientation_deviation_max > 0.0)
        {
          double dtheta = angles::normalize_angle(marker_new.theta - first_in_odom_.theta);
          if (config_.orientation_deviation_max < std::abs(dtheta * 180.0 / M_PI)) continue;
        }
      }

      if (dist < 0 || d < dist)
      {
        dist = d;
        marker_in_odom = marker_new;
        ret = marker;
      }
    }

    if (dist < 0) return ret;

    publishTransform(marker_header, marker_in_odom);
    if (!has_first_target_)
    {
      has_first_target_ = true;
      first_in_odom_ = marker_in_odom;
    }
    return ret;
  }

  virtual geometry_msgs::Pose2D handleMarkerPairs(const geometry_msgs::PoseArray& markers, double min_separation,
                                                  double max_separation, double max_position_deviation)
  {
    // Compute center as target
    std::vector<geometry_msgs::Pose2D> markers_2d;

    for (int i = (markers.poses.size() < 3); i < markers.poses.size(); i++)
    {
      int j = i ? i - 1: markers.poses.size() - 1;

      // Check separation distance
      double d = poseDistance(markers.poses[i], markers.poses[j]);
      if (d < min_separation || d > max_separation) continue;

      geometry_msgs::Pose2D marker_new;
      getMarkerInCenter(std::complex<double>(markers.poses[i].position.x, markers.poses[i].position.y),
                        std::complex<double>(markers.poses[j].position.x, markers.poses[j].position.y),
                        marker_new);

      markers_2d.push_back(marker_new);
    }

    return handleMarkers(markers.header, markers_2d, max_position_deviation);
  }

protected:
  tf2_ros::Buffer& tf_;
  tf2_ros::TransformBroadcaster& tfb_;

  ros::Publisher& cloud_flatten_pub_;
  ros::Publisher& landmarks_pub_;

  marker_localization::MarkerLocalizationConfig& config_;
  std::string sensor_;
  uint8_t profile_;

  geometry_msgs::Pose2D marker_in_odom_;
  MeanFilter<double> marker_x_;
  MeanFilter<double> marker_y_;
  MeanFilter<double> marker_theta_;

  geometry_msgs::Pose2D first_in_odom_;
  bool has_first_target_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTOR_H
