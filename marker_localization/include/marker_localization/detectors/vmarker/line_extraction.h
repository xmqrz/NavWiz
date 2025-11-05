/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include <cmath>
#include <vector>
#include <boost/array.hpp>
#include <eigen3/Eigen/Dense>
#include <marker_localization/detectors/vmarker/utilities.h>

#ifndef MARKER_LOCALIZATION_DETECTORS_VMARKER_LINE_EXTRACTION_H
#define MARKER_LOCALIZATION_DETECTORS_VMARKER_LINE_EXTRACTION_H

namespace marker_localization
{

class Line
{
public:
  // Constructor / destructor
  Line(const CachedData&, const RangeData&, const Params&, std::vector<unsigned int>);
  Line(double angle, double radius, const boost::array<double, 4> &covariance,
       const boost::array<double, 2> &start, const boost::array<double, 2> &end,
       const std::vector<unsigned int> &indices);
  ~Line();
  // Get methods for the line parameters
  double                           getAngle() const;
  const boost::array<double, 4>&   getCovariance() const;
  const boost::array<double, 2>&   getEnd() const;
  const std::vector<unsigned int>& getIndices() const;
  double                           getRadius() const;
  const boost::array<double, 2>&   getStart() const;
  // Methods for line fitting
  double       distToPoint(unsigned int);
  void         endpointFit();
  void         leastSqFit();
  double       length() const;
  unsigned int numPoints() const;
  void         projectEndpoints();

private:
  std::vector<unsigned int> indices_;
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  PointParams p_params_;
  // Point variances used for least squares
  std::vector<double> point_scalar_vars_;
  std::vector<boost::array<double, 4> > point_covs_;
  double p_rr_;
  // Line parameters
  double angle_;
  double radius_;
  boost::array<double, 2> start_;
  boost::array<double, 2> end_;
  boost::array<double, 4> covariance_;
  // Methods
  void    angleFromEndpoints();
  void    angleFromLeastSq();
  double  angleIncrement();
  void    calcCovariance();
  void    calcPointCovariances();
  void    calcPointParameters();
  void    calcPointScalarCovariances();
  void    radiusFromEndpoints();
  void    radiusFromLeastSq();
};  // class Line


class LineExtraction
{
public:
  // Constructor / destructor
  LineExtraction();
  ~LineExtraction();
  // Run
  void extractLines(std::vector<Line>& lines);
  // Data setting
  void setCachedData(const std::vector<double>& bearings, const std::vector<double>& cos_bearings,
                     const std::vector<double>& sin_bearings, const std::vector<unsigned int>& indices);
  void setRangeData(const std::vector<double>& ranges);
  // Parameter setting
  void setBearingVariance(double value);
  void setRangeVariance(double value);
  void setLeastSqAngleThresh(double value);
  void setLeastSqRadiusThresh(double value);
  void setMaxLineGap(double value);
  void setMinLineLength(double value);
  void setMinLinePoints(unsigned int value);
  void setMinRange(double value);
  void setMaxRange(double value);
  void setMinSplitDist(double value);
  void setOutlierDist(double value);

private:
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void   filterCloseAndFarPoints();
  void   filterOutlierPoints();
  void   filterLines();
  void   mergeLines();
  void   split(const std::vector<unsigned int>& indices);
};  // class LineExtraction


}  // namespace marker_localization

#endif  // MARKER_LOCALIZATION_DETECTORS_VMARKER_LINE_EXTRACTION_H
