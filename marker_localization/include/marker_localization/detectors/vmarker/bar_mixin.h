/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_VMARKER_BAR_MIXIN_H
#define MARKER_LOCALIZATION_DETECTORS_VMARKER_BAR_MIXIN_H

#include <marker_localization/detectors/vmarker/line_extraction.h>
#include <marker_localization/marker_localization.h>
#include <marker_localization/utils.h>

#include <sensor_msgs/LaserScan.h>


namespace marker_localization
{

class BarMixin
{
public:
  explicit BarMixin(const marker_localization::MarkerLocalizationConfig& config,
                    const int& index_min, const int& index_max);
  void start(float range_max);

protected:
  void cacheData(const sensor_msgs::LaserScan& scan);
  void processData(const sensor_msgs::LaserScan& scan,
                   const geometry_msgs::PoseArray& markers,
                   std::vector<geometry_msgs::Pose2D>& markers_2d,
                   std::set<size_t>& indices);

protected:
  LineExtraction line_extraction_;

  const int& trim_index_min_;
  const int& trim_index_max_;

  // bar config
  struct
  {
    const double *min_angle;
    const double *max_angle;
    const double *min_length;
    const double *max_length;
    const double *max_gap;
  } config_bar_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_VMARKER_BAR_MIXIN_H
