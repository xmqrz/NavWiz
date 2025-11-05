/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include "marker_localization/detectors/vmarker/v_extraction.h"
#include <marker_localization/utils.h>


namespace marker_localization
{
VExtraction::VExtraction()
{
}

VExtraction::~VExtraction()
{
}

void VExtraction::getMarkers(const std::vector<Line>& lines, Pose2DList& markers, std::set<size_t>& line_indices)
{
  markers.clear();

  if (lines.size() < 2)
  {
    return;
  }

  extract(lines, markers, line_indices);
}

void VExtraction::extractVL(const std::vector<Line>& lines, Pose2DList& v_list, Pose2DList& l_list, Pose2DList& vl_list,
                            std::set<size_t>& v_lines, std::set<size_t>& l_lines, std::set<size_t>& vl_lines)
{
  int vl_flag = 0;

  for (size_t i = 0; i < lines.size(); i++)
  {
    size_t l = i ? i - 1 : lines.size() - 1;
    const Line& line = lines[l];
    const Line& next = lines[i];

    if (calcDist(line.getEnd(), next.getStart()) < config_.max_intercept_distance)
    {
      Coordinate a, b, c, d;
      a = line.getStart();  // a     d
      b = line.getEnd();    //  \   /
      c = next.getStart();  //   b c
      d = next.getEnd();

      double lenghtAB = line.length();
      double lenghtCD = next.length();
      if (lenghtAB > config_.max_length || lenghtAB < config_.min_length ||
          lenghtCD > config_.max_length || lenghtCD < config_.min_length)
      {
        continue;
      }

      tf2::Vector3 vBA(a[0] - b[0], a[1] - b[1], 0),
          vCD(d[0] - c[0], d[1] - c[1], 0);

      tf2::Vector3 crossproduct = vBA.cross(vCD);
      double direction = crossproduct.getZ();

      double angle = vBA.angle(vCD);

      double degree = angle * 180.0 / M_PI;

      // outward(v) > 0, inward(^) < 0
      if (direction > 0)
      {
        if (degree > config_.max_l_angle || degree < config_.min_l_angle)
        {
          continue;
        }
      }
      else
      {
        if (degree > config_.max_v_angle || degree < config_.min_v_angle)
        {
          continue;
        }
      }

      double m1 = (b[1] - a[1]) / (b[0] - a[0]);
      double c1 = a[1] - m1 * a[0];
      double m2 = (d[1] - c[1]) / (d[0] - c[0]);
      double c2 = c[1] - m2 * c[0];

      double x = (c2 - c1) / (m1 - m2);
      double y = m1 * x + c1;

      // sum of unit vector
      tf2::Vector3 sum = (direction > 0)? -vBA.normalized() - vCD.normalized():
                                           vBA.normalized() + vCD.normalized();
      double yaw = atan2(sum.getY(), sum.getX());

      geometry_msgs::Pose2D v;
      v.x = x;
      v.y = y;
      v.theta = yaw;

      if (direction > 0)
      {
        l_list.push_back(v);
        l_lines.insert(l);
        l_lines.insert(i);
      }
      else
      {
        v_list.push_back(v);
        v_lines.insert(l);
        v_lines.insert(i);
      }

      if (vl_flag && ((direction > 0) ^ (vl_flag > 0)))
      {
        double dist = poseDistance(v_list.back(), l_list.back());
        if (dist > config_.min_length && dist < config_.max_length)
        {
          vl_list.push_back(v_list.back());
          vl_lines.insert(l ? l - 1 : lines.size() - 1);
          vl_lines.insert(l);
          vl_lines.insert(i);
        }
      }

      vl_flag = (direction > 0)? 1: -1;
    }
  }
}

void VExtraction::extractPair(const Pose2DList& list, Pose2DList& pairs)
{
  for (int i = 1; i < list.size(); i++)
  {
    int k = i - 1;
    for (int j = i; j < list.size(); j++)
    {
      // Check separation distance
      double dist = poseDistance(list[j], list[k]);
      if (dist < config_.min_separation || dist > config_.max_separation)
      {
        continue;
      }

      geometry_msgs::Pose2D marker_new;
      getMarkerInCenter(std::complex<double>(list[j].x, list[j].y),
                        std::complex<double>(list[k].x, list[k].y),
                        marker_new);
      pairs.push_back(marker_new);
    }
  }
}


double VExtraction::calcDist(const Coordinate& a, const Coordinate& b)
{
  return sqrt(pow(b[1] - a[1], 2) + pow(b[0] - a[0], 2));
}


void VExtraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  Pose2DList list_a, list_b;
  std::set<size_t> line_a, line_b;
  extractVL(lines, list, list_a, list_b, line_indices, line_a, line_b);
}

void LExtraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  Pose2DList list_a, list_b;
  std::set<size_t> line_a, line_b;
  extractVL(lines, list_a, list, list_b, line_a, line_indices, line_b);
}

void VLExtraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  Pose2DList list_a, list_b;
  std::set<size_t> line_a, line_b;
  extractVL(lines, list_a, list_b, list, line_a, line_b, line_indices);
}

void V2Extraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  Pose2DList list_a, list_b, list_v;
  std::set<size_t> line_a, line_b;
  extractVL(lines, list_v, list_a, list_b, line_indices, line_a, line_b);
  extractPair(list_v, list);
}

void L2Extraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  Pose2DList list_a, list_b, list_l;
  std::set<size_t> line_a, line_b;
  extractVL(lines, list_a, list_l, list_b, line_a, line_indices, line_b);
  extractPair(list_l, list);
}

void BarExtraction::extract(const std::vector<Line>& lines, Pose2DList& list, std::set<size_t>& line_indices)
{
  double dot_threshold = cos(config_.max_v_angle * M_PI / 180.0);
  std::vector<Line> bars;

  for (std::vector<Line>::const_iterator it = lines.begin(); it != lines.end(); it++)
  {
    // Check Bar length
    if (it->length() > config_.min_length && it->length() < config_.max_length)
    {
      bars.push_back(*it);
    }
  }

  for (int i = 1; i < bars.size(); i++)
  {
    Coordinate a, b;
    a = bars[i - 1].getStart();
    b = bars[i - 1].getEnd();

    tf2::Vector3 vA(a[0], a[1], 0);
    tf2::Vector3 vB(b[0], b[1], 0);
    if (vB.length2() < vA.length2())
    {
      tf2::Vector3 v = vA;
      vA = vB;
      vB = v;
    }
    tf2::Vector3 uBA = (vA - vB).normalize();

    for (int j = i; j < bars.size(); j++)
    {
      Coordinate c, d;
      c = bars[j].getStart();
      d = bars[j].getEnd();

      tf2::Vector3 vC(c[0], c[1], 0);
      tf2::Vector3 vD(d[0], d[1], 0);
      if (vC.length2() < vD.length2())
      {
        tf2::Vector3 v = vC;
        vC = vD;
        vD = v;
      }
      tf2::Vector3 uCD = (vD - vC).normalize();
      // a     d
      //  \   /
      //   b c

      // Check separation angle
      if (uBA.dot(uCD) < dot_threshold)
      {
        continue;
      }

      // Check separation distance
      tf2::Vector3 sum = (uBA + uCD).normalize();
      double dist = std::abs(sum.cross(vA - vD).getZ());
      if (dist < config_.min_separation || dist > config_.max_separation)
      {
        continue;
      }

      geometry_msgs::Pose2D marker_new;
      marker_new.theta = atan2(sum.getY(), sum.getX());
      sum = (head_ ? (vA + vD) : (vB + vC)) * 0.5;
      marker_new.x = sum.getX();
      marker_new.y = sum.getY();
      list.push_back(marker_new);
      line_indices.insert(i - 1);
      line_indices.insert(j);
    }
  }
}


}  // namespace marker_localization
