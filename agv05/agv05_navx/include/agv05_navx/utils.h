/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef AGV05_NAVX_UTILS_H
#define AGV05_NAVX_UTILS_H

#include <agv05_nav/utils.h>

#include <geometry_msgs/Polygon.h>
#include <ros/time.h>


namespace agv05
{

template<class T>
T getCurvature(std::complex<T> prime, std::complex<T> prime2)
{
  T d = std::abs(prime);
  return (prime.real() * prime2.imag() - prime.imag() * prime2.real()) / (d * d * d);
}

// https://stackoverflow.com/questions/2009160/how-do-i-convert-the-2-control-points-of-a-cubic-curve-to-the-single-control-poi/#14514491
inline double getCurvature(const geometry_msgs::Polygon& path)
{
  auto curvature = [](std::complex<double> m, std::complex<double> d, std::complex<double> c) -> double
  {
    // B(t) = 0.5at² + bt + P(0)
    double mc = cross(m, c);
    if (!mc) return 0.0;
    std::complex<double> b = m * (cross(d, c) * 2.0 / mc);
    std::complex<double> a = (d - b) * 2.0;
    double aa = std::norm(a);
    if (!aa) return 0.0;
    double t = -dot(a, b) / aa;
    return std::abs(getCurvature((t < 0.0 ? 0.0 : t > 1.0 ? 1.0 : t) * a + b, a));
  };

  if (path.points.size() == 3)
  {
    std::complex<double> p1(path.points[0].x, path.points[0].y);
    std::complex<double> c(path.points[1].x, path.points[1].y);
    std::complex<double> p2(path.points[2].x, path.points[2].y);
    return curvature(c - p1, p2 - p1, p2 - c);
  }

  if (path.points.size() != 4) return 0.0;

  std::complex<double> p1(path.points[0].x, path.points[0].y);
  std::complex<double> c1(path.points[1].x, path.points[1].y);
  std::complex<double> c2(path.points[2].x, path.points[2].y);
  std::complex<double> p2(path.points[3].x, path.points[3].y);

  // B(t) = at³ - 3bt² + 3ct + P(0)
  std::complex<double> d = c2 - c1;
  std::complex<double> c = c1 - p1;
  std::complex<double> b = c - d;
  std::complex<double> a = p2 - p1 - d * 3.0;

  double tr = 0.5;
  double ab = cross(a, b);
  double ac = cross(a, c);
  double bc = cross(b, c);

  if (ab)
  {
    ac *= 0.5;
    double rr = ac * ac - ab * bc;
    if (rr >= 0.0)
    {
      double r = std::sqrt(rr);
      double t = (ac + r) / ab;
      if (t > 0.0 && t < 1.0)
      {
        tr = t;
      }
      else if (r)
      {
        t = (ac - r) / ab;
        if (t > 0.0 && t < 1.0)
        {
          tr = t;
        }
      }
    }
  }
  else if (ac)
  {
    double t = bc / ac;
    if (t > 0.0 && t < 1.0)
    {
      tr = t;
    }
  }
  else return 0.0;

  std::complex<double> pt = tr * tr * tr * a - 3.0 * tr * (tr * b - c) + p1;
  std::complex<double> m = (tr * tr * a - 2.0 * tr * b + c) * 3.0;
  return std::max(curvature(m, p1 - pt, c == 0.0 ? d : c), curvature(m, p2 - pt, p2 == c2 ? d : p2 - c2));
}

inline double getBezierLength(const geometry_msgs::Polygon& path)
{
  if (path.points.size() < 2)
  {
    return 0.0;
  }

  /*
   * Jens Gravesen - the best estimate of the Bézier arc-length is
   *       [ 2Lc + (n-1)Lp ] / (n+1)
   * where n is the degree of the Bézier curve
   *       Lc denotes the total chord-length of the pieces and
   *       Lp denotes the total polygon-length of the pieces
   */

  std::complex<double> points[2] =
  {
    std::complex<double>(path.points[0].x, path.points[0].y),
    std::complex<double>(path.points[path.points.size() - 1].x, path.points[path.points.size() - 1].y),
  };
  double Lc = std::abs(points[1] - points[0]);

  if (path.points.size() > 2)
  {
    double Lp = 0.0;
    for (std::size_t i = 1; i < path.points.size(); i++)
    {
      points[i & 1] = std::complex<double>(path.points[i].x, path.points[i].y);
      Lp += std::abs(points[1] - points[0]);
    }

    return ((Lc * 2 + Lp * (path.points.size() - 2)) / path.points.size());
  }

  return Lc;
}


/* Adaptation of ros::Rate. Provide next cycle start time */
class Rate
{
public:
  explicit Rate(double frequency) :
    start_(ros::Time::now()),
    expected_cycle_time_(1.0 / frequency)
  {}

  ros::Time nextCycle()
  {
    ros::Time expected_end = start_ + expected_cycle_time_;
    ros::Time actual_end = ros::Time::now();

    // detect backward jumps in time
    if (actual_end < start_)
    {
      expected_end = actual_end + expected_cycle_time_;
    }

    // make sure to reset our start time
    start_ = expected_end;

    // if we've jumped forward in time, or the loop has taken more than a
    // full extra cycle, reset our cycle
    if (actual_end > expected_end + expected_cycle_time_)
    {
      start_ = actual_end;
    }

    return expected_end;
  }

  void reset(double frequency = 0.0)
  {
    start_ = ros::Time::now();
    if (frequency > 0.0)
    {
      expected_cycle_time_ = ros::Duration(1.0 / frequency);
    }
  }

  ros::Duration expectedCycleTime() const { return expected_cycle_time_; }

private:
  ros::Time start_;
  ros::Duration expected_cycle_time_;
};

}  // namespace agv05

#endif  // AGV05_NAVX_UTILS_H
