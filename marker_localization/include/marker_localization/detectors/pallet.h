/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_PALLET_H
#define MARKER_LOCALIZATION_DETECTORS_PALLET_H

#include <geometry_msgs/Pose2D.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <marker_localization/detector.h>
#include <pcl_ros/point_cloud.h>


namespace marker_localization
{

class PalletDetector: public Detector
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud0;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

public:
  PalletDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile) :
    Detector(ml, sensor, profile)
  {
    switch (profile)
    {
#define _LOAD_PROFILE(p) \
  config_pallet_.crop_z_min                          = &config_.pallet_crop_z_min##p; \
  config_pallet_.crop_z_max                          = &config_.pallet_crop_z_max##p; \
  config_pallet_.block_height_min                    = &config_.pallet_block_height_min##p; \
  config_pallet_.block_height_max                    = &config_.pallet_block_height_max##p; \
  config_pallet_.block_width_min                     = &config_.pallet_block_width_min##p; \
  config_pallet_.block_width_max                     = &config_.pallet_block_width_max##p; \
  config_pallet_.block_vertical_angle_min            = &config_.pallet_block_vertical_angle_min##p; \
  config_pallet_.block_vertical_angle_max            = &config_.pallet_block_vertical_angle_max##p; \
  config_pallet_.block_smoothness_angle              = &config_.pallet_block_smoothness_angle##p; \
  config_pallet_.pocket_count                        = &config_.pallet_pocket_count##p; \
  config_pallet_.blocks_distance_min                 = &config_.pallet_blocks_distance_min##p; \
  config_pallet_.blocks_distance_max                 = &config_.pallet_blocks_distance_max##p; \
  config_pallet_.blocks_alignment_angle              = &config_.pallet_blocks_alignment_angle##p; \
  config_pallet_.center_block_line_fitting_max_error = &config_.pallet_center_block_line_fitting_max_error##p; \
  config_pallet_.deck_line_fitting_max_error         = &config_.pallet_deck_line_fitting_max_error##p; \
  config_pallet_.first_position_max                  = &config_.pallet_first_position_max##p; \
  config_pallet_.first_orientation_max               = &config_.pallet_first_orientation_max##p; \
  config_pallet_.position_deviation_max              = &config_.pallet_position_deviation_max##p

    default:
    case 1: _LOAD_PROFILE(); break;
    case 2: _LOAD_PROFILE(_2); break;
    case 3: _LOAD_PROFILE(_3); break;
    case 4: _LOAD_PROFILE(_4); break;
    case 5: _LOAD_PROFILE(_5); break;

#undef _LOAD_PROFILE
    }
  }

  void start();
  void stop();

private:
  // void callbackPointCloud(const PointCloud::ConstPtr& cloud);
  void callbackDepth(const sensor_msgs::ImageConstPtr& depth);

  bool voxelFilter(const sensor_msgs::ImageConstPtr& depth, const tf2::Transform& camera_tf);
  bool verticalFilter(const float camera_x);
  bool cluster(const float camera_x);
  bool fitBlockLine(Eigen::Vector2f& center, Eigen::Vector2f& gradient, Eigen::Vector2f& c0,
                    std::vector<Eigen::Vector2f>& cc1, std::vector<Eigen::Vector2f>& cc2);
  bool fitPalletDeck(int column_start, int column_end, Eigen::Vector2f center, Eigen::Vector2f gradient);
  void addTarget(int column_start, int column_end, Eigen::Vector2f center, Eigen::Vector2f gradient,
                 const float camera_x, std::vector<geometry_msgs::Pose2D>& targets);
  void sendTarget(const std::vector<geometry_msgs::Pose2D>& targets, const ros::Time& timestamp);

private:
  static ros::Publisher cloud_voxel_pub_;
  static ros::Publisher cloud_vertical_pub_;
  static ros::Publisher cloud_colored_pub_;
  static ros::Publisher cloud_deck_pub_;
  static ros::Publisher marker_viz_pub_;
  static image_transport::Publisher depth_voxel_pub_;
  static image_transport::Publisher depth_vertical_pub_;

  image_transport::Subscriber depth_sub_;

  image_geometry::PinholeCameraModel model_;

  sensor_msgs::Image depth_voxel_;
  sensor_msgs::Image depth_vertical_;
  PointCloud0::Ptr cloud_voxel_;
  PointCloud0::Ptr cloud_vertical_;
  PointCloud0::Ptr cloud_deck_;
  int voxel_count_;
  int vertical_count_;

  float trim_angle_min_;
  float trim_angle_max_;
  uint16_t trim_range_max_;

  Eigen::Vector2f first_target_;
  double first_target_yaw_;

  // pallet config
  struct
  {
    const double *crop_z_min;
    const double *crop_z_max;
    const double *block_height_min;
    const double *block_height_max;
    const double *block_width_min;
    const double *block_width_max;
    const double *block_vertical_angle_min;
    const double *block_vertical_angle_max;
    const double *block_smoothness_angle;
    const int    *pocket_count;
    const double *blocks_distance_min;
    const double *blocks_distance_max;
    const double *blocks_alignment_angle;
    const double *center_block_line_fitting_max_error;
    const double *deck_line_fitting_max_error;
    const double *first_position_max;
    const double *first_orientation_max;
    const double *position_deviation_max;
  } config_pallet_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_PALLET_H
