/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#ifndef MARKER_LOCALIZATION_DETECTORS_VMARKER_H
#define MARKER_LOCALIZATION_DETECTORS_VMARKER_H

#include <marker_localization/detector.h>
#include <marker_localization/detectors/vmarker/line_extraction.h>
#include <marker_localization/detectors/vmarker/v_extraction.h>
#include <tf2/LinearMath/Transform.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>


namespace marker_localization
{

class VMarkerDetector: public Detector
{
public:
  explicit VMarkerDetector(MarkerLocalization& ml, const std::string& sensor, uint8_t profile, boost::shared_ptr<VExtraction> extract) :
    Detector(ml, sensor, profile),
    marker_extraction_(extract),
    data_cached_(false)
  {
    if (!marker_extraction_)
    {
      marker_extraction_ = boost::make_shared<VExtraction>();
    }
  }

  virtual ~VMarkerDetector()
  {
    marker_extraction_.reset();
  }

  void start();
  void stop();

private:
  void callbackLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);

  // visualization
  void populateLineViz(const sensor_msgs::LaserScanConstPtr& laser_scan,
                       const std::vector<Line>& lines, const std::set<size_t>& line_indices);
  void populateLineViz(const std::vector<Line>& lines, visualization_msgs::Marker& marker_msg);
  void populateVViz(const Pose2DList& vlist, visualization_msgs::Marker& marker_msg);

  // line marker
  void cacheData(const sensor_msgs::LaserScanConstPtr& scan_msg);

protected:
  boost::shared_ptr<VExtraction> marker_extraction_;

private:
  static ros::Publisher marker_viz_pub_;
  ros::Subscriber laser_sub_;

  LineExtraction line_extraction_;

  bool data_cached_;
  float trim_angle_min_;
  float trim_angle_max_;
  int trim_index_min_;
  int trim_index_max_;

  // v marker config
  double max_trans_dev_;
  bool pub_viz_;

  // diagnostic
  uint32_t diagnostic_line_detected_;
  uint32_t diagnostic_v_detected_;
  std::string diagnostic_v_x_;
  std::string diagnostic_v_y_;
};

}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_VMARKER_H
