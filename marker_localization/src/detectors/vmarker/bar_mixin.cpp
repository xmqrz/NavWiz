/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <angles/angles.h>
#include <marker_localization/detectors/vmarker/bar_mixin.h>


namespace marker_localization
{

BarMixin::BarMixin(const marker_localization::MarkerLocalizationConfig& config,
                   const int& index_min, const int& index_max) :
  trim_index_min_(index_min), trim_index_max_(index_max)
{
  // Line Extraction config
  line_extraction_.setBearingVariance(config.bearing_std_dev * config.bearing_std_dev);
  line_extraction_.setRangeVariance(config.range_std_dev * config.range_std_dev);
  line_extraction_.setLeastSqAngleThresh(config.least_sq_angle_thresh);
  line_extraction_.setLeastSqRadiusThresh(config.least_sq_radius_thresh);
  line_extraction_.setMaxLineGap(config.max_line_gap);
  line_extraction_.setMinLineLength(config.min_line_length);
  line_extraction_.setMinRange(config.min_range);
  line_extraction_.setMaxRange(config.max_range);
  line_extraction_.setMinSplitDist(config.min_split_dist);
  line_extraction_.setOutlierDist(config.outlier_dist);
  line_extraction_.setMinLinePoints(config.min_line_points);
}

void BarMixin::start(float range_max)
{
  if (range_max < std::numeric_limits<float>::max())
  {
    line_extraction_.setMaxRange(range_max);
  }
}

void BarMixin::cacheData(const sensor_msgs::LaserScan& scan)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;

  int i = trim_index_min_;
  int n = (trim_index_min_ && (trim_index_min_ >= trim_index_max_)) ?
          static_cast<int>(scan.ranges.size()) : trim_index_max_;

  for (int count = 0; i < n; ++count)
  {
    const double b = scan.angle_min + i * scan.angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(std::cos(b));
    sin_bearings.push_back(std::sin(b));
    indices.push_back(count);

    if (++i == static_cast<int>(scan.ranges.size()))
    {
      if (trim_index_min_ && (trim_index_min_ >= trim_index_max_))
      {
        i = 0;
        n = trim_index_max_;
      }
    }
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

void BarMixin::processData(const sensor_msgs::LaserScan& scan,
                           const geometry_msgs::PoseArray& markers,
                           std::vector<geometry_msgs::Pose2D>& markers_2d,
                           std::set<size_t>& indices)
{
  ROS_ASSERT(scan.header.frame_id == markers.header.frame_id);

  int i = trim_index_min_;
  int n = (trim_index_min_ && (trim_index_min_ >= trim_index_max_)) ?
          static_cast<int>(scan.ranges.size()) : trim_index_max_;

  if (i >= n)
  {
    return;
  }

  // line extraction
  std::vector<double> scan_ranges_doubles(scan.ranges.begin() + i, scan.ranges.begin() + n);
  if (trim_index_min_ && (trim_index_min_ >= trim_index_max_))
  {
    scan_ranges_doubles.insert(scan_ranges_doubles.end(), scan.ranges.begin(), scan.ranges.begin() + trim_index_max_);
  }
  line_extraction_.setRangeData(scan_ranges_doubles);

  std::vector<int> line_indices;
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);
  for (int i = 0, n = lines.size(); i < n; i++)
  {
    const Line& line = lines[i];

    double yaw = line.getAngle() * 180 / M_PI + 180;
    if (yaw > 180 && (*config_bar_.min_angle < 0 || *config_bar_.max_angle < 180))
    {
      yaw -= 360;
    }
    if (yaw < *config_bar_.min_angle) continue;
    if (yaw > *config_bar_.max_angle) continue;

    double len = line.length();
    if (len < *config_bar_.min_length) continue;
    if (len > *config_bar_.max_length) continue;

    line_indices.push_back(i);
  }

  // marker extraction
  for (const auto& p : markers.poses)
  {
    std::complex<double> r(p.position.x, p.position.y);
    double range = std::abs(r);
    double angle = std::arg(r);

    int count = 0;
    double theta, dtheta = 0;
    for (auto l : line_indices)
    {
      const Line& line = lines[l];
      double dist = std::abs(range * std::cos(angle - line.getAngle()) - line.getRadius());
      if (dist < *config_bar_.max_gap)
      {
        boost::array<double, 2> a = line.getStart(), b = line.getEnd();
        std::complex<double> ra = std::complex<double>(a[0], a[1]) - r;
        std::complex<double> rb = std::complex<double>(b[0], b[1]) - r;
        dist = *config_bar_.max_gap;
        if (dot(ra, rb) < 0 || std::min(std::norm(ra), std::norm(rb)) < (dist * dist))
        {
          if (count++)
          {
            dtheta += angles::normalize_angle(line.getAngle() - theta);
          }
          else
          {
            theta = line.getAngle();
          }

          for (auto i : line.getIndices())
          {
            size_t n = i + trim_index_min_;
            indices.insert(n > scan.ranges.size() ? n - scan.ranges.size() : n);
          }
        }
      }
    }
    if (count)
    {
      if (count > 1)
      {
        theta += (dtheta / count);
      }

      geometry_msgs::Pose2D marker;
      marker.x = p.position.x;
      marker.y = p.position.y;
      marker.theta = angles::normalize_angle(theta + M_PI);
      markers_2d.push_back(marker);
    }
  }
}

}  // namespace marker_localization
