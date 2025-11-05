/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include <geometry_msgs/Pose2D.h>

#include <marker_localization/MarkerLocalizationConfig.h>

#include <marker_localization/detectors/vmarker/line_extraction.h>

#ifndef MARKER_LOCALIZATION_DETECTORS_VMARKER_V_EXTRACTION_H
#define MARKER_LOCALIZATION_DETECTORS_VMARKER_V_EXTRACTION_H

namespace marker_localization
{

typedef boost::array<double, 2> Coordinate;
typedef std::vector<geometry_msgs::Pose2D> Pose2DList;


#define _SET_CONFIG_PROFILE(t, l, v, s, p) \
  config_.min_v_angle = config.min_##v##_angle##p; \
  config_.max_v_angle = config.max_##v##_angle##p; \
  config_.min_l_angle = config.min_##l##_angle##p; \
  config_.max_l_angle = config.max_##l##_angle##p; \
  config_.min_length = config.min_##t##_length##p; \
  config_.max_length = config.max_##t##_length##p; \
  config_.min_separation = config.min_##t##_##s##p; \
  config_.max_separation = config.max_##t##_##s##p; \
  config_.max_intercept_distance = config.max_intercept_distance; \
  return config.max_##t##_translational_deviation##p

#define _SET_CONFIG(t, l, v, s, p) \
  switch (p)                                  \
  {                                           \
  case 5: _SET_CONFIG_PROFILE(t, l, v, s, 5); \
  case 4: _SET_CONFIG_PROFILE(t, l, v, s, 4); \
  case 3: _SET_CONFIG_PROFILE(t, l, v, s, 3); \
  case 2: _SET_CONFIG_PROFILE(t, l, v, s, 2); \
  case 1:                                     \
  default: break;                             \
  }                                           \
  _SET_CONFIG_PROFILE(t, l, v, s, 1)


class VExtraction
{
public:
  // Constructor / destructor
  VExtraction();
  virtual ~VExtraction();
  void getMarkers(const std::vector<Line>& lines, Pose2DList& markers, std::set<size_t>& line_indices);

  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
    _SET_CONFIG(v, v, v, length, profile);
  }

protected:
  void extractVL(const std::vector<Line>& lines, Pose2DList& v_list, Pose2DList& l_list, Pose2DList& vl_list,
                 std::set<size_t>& v_lines, std::set<size_t>& l_lines, std::set<size_t>& vl_lines);
  void extractPair(const Pose2DList& list, Pose2DList& pairs);

private:
  double calcDist(const Coordinate& a, const Coordinate& b);

  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);

protected:
  struct
  {
    double min_v_angle;
    double max_v_angle;
    double min_l_angle;
    double max_l_angle;
    double min_length;
    double max_length;
    double min_separation;
    double max_separation;
    double max_intercept_distance;
  } config_;
};  // class VExtraction

class LExtraction: public VExtraction
{
public:
  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
    _SET_CONFIG(l, l, l, length, profile);
  }

private:
  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);
};  // class LExtraction

class VLExtraction: public VExtraction
{
public:
  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
    _SET_CONFIG(vl, vl_l, vl_v, length, profile);
  }

private:
  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);
};  // class VLExtraction

class V2Extraction: public VExtraction
{
public:
  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
    _SET_CONFIG(v2, v2, v2, separation, profile);
  }

private:
  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);
};  // class V2Extraction

class L2Extraction: public VExtraction
{
public:
  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
    _SET_CONFIG(l2, l2, l2, separation, profile);
  }

private:
  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);
};  // class L2Extraction

class BarExtraction: public VExtraction
{
public:
  explicit BarExtraction(bool head = true) : VExtraction(), head_(head) {}

  virtual const double setConfig(const marker_localization::MarkerLocalizationConfig& config, uint8_t profile)
  {
#define min_bar_pair_angle1 max_bar_pair_angle1
#define min_bar_pair_angle2 max_bar_pair_angle2
#define min_bar_pair_angle3 max_bar_pair_angle3
#define min_bar_pair_angle4 max_bar_pair_angle4
#define min_bar_pair_angle5 max_bar_pair_angle5
    _SET_CONFIG(bar, bar_pair, bar_pair, separation, profile);
#undef min_bar_pair_angle1
#undef min_bar_pair_angle2
#undef min_bar_pair_angle3
#undef min_bar_pair_angle4
#undef min_bar_pair_angle5
  }

private:
  virtual void extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices);

  const bool head_;
};  // class BarExtraction

#undef _SET_CONFIG
#undef _SET_CONFIG_PROFILE


}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_VMARKER_V_EXTRACTION_H
