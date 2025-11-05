/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#ifndef MARKER_LOCALIZATION_UTILS_H
#define MARKER_LOCALIZATION_UTILS_H

#include <angles/angles.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace marker_localization
{

template<class T>
class MeanFilter
{
public:
  explicit MeanFilter(size_t count = 0, const T& value = T()) : data_(count, value)
  {
    index_ = 0;
    sum_ = value * count;
  }

  T update(T value)
  {
    index_ = (index_ ? index_ : data_.size()) - 1;
    sum_ += (value - data_[index_]);
    data_[index_] = value;
    return sum_ / data_.size();
  }

protected:
  std::vector<T> data_;
  size_t index_;
  T sum_;
};

template<class T>
T dot(const std::complex<T>& a, const std::complex<T>& b)
{
  return a.real() * b.real() + a.imag() * b.imag();
}

template<class T>
T cross(const std::complex<T>& a, const std::complex<T>& b)
{
  return a.real() * b.imag() - a.imag() * b.real();
}

template<class T>
void getMarkerInCenter(const std::complex<T>& a, const std::complex<T>& b, geometry_msgs::Pose2D& marker)
{
  std::complex<T> c = (a + b) * std::complex<T>(0.5, 0);
  std::complex<T> v = b - a;
  std::complex<T> vp = std::complex<T>(-v.imag(), v.real());
  if (dot(vp, c) > 0)
  {
    vp = std::complex<T>(v.imag(), -v.real());
  }

  marker.x = c.real();
  marker.y = c.imag();
  marker.theta = std::arg(vp);
}

inline double poseDistance(const geometry_msgs::Pose2D& a, const geometry_msgs::Pose2D& b)
{
  return std::abs(std::complex<double>(a.x, a.y) - std::complex<double>(b.x, b.y));
}

inline double poseDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline void getTrimIndex(int size, float angle_min, float angle_increment,
                         float trim_angle_min, float trim_angle_max,
                         int& trim_index_min, int& trim_index_max)
{
  if (angle_min < 0)  // -π -> +π
  {
    trim_angle_min = angles::normalize_angle(trim_angle_min);
    trim_angle_max = angles::normalize_angle(trim_angle_max);
  }
  else  // 0 -> 2π
  {
    trim_angle_min = angles::normalize_angle_positive(trim_angle_min);
    trim_angle_max = angles::normalize_angle_positive(trim_angle_max);
  }

  trim_index_min = (trim_angle_min - angle_min) / angle_increment;
  trim_index_max = (trim_angle_max - angle_min) / angle_increment;

  if (fabs(angles::normalize_angle(trim_angle_max - trim_angle_min)) < angle_increment)
  {
    if (0 < trim_index_min && trim_index_min < size)
    {
      trim_index_max = trim_index_min;
    }
    else
    {
      trim_index_min = 0;
      trim_index_max = size;
    }
  }
  else
  {
    trim_index_min = std::max(trim_index_min, 0);
    trim_index_max = std::min(trim_index_max + 1, size);

    if (trim_angle_min > trim_angle_max)
    {
      if (trim_index_min >= size)
      {
        trim_index_min = 0;
        if (trim_angle_max < angle_min)
        {
          // trim angle range is not in sensor range
          trim_index_max = 0;
        }
      }
      else if (trim_angle_max < angle_min)
      {
        trim_index_max = size;
      }
    }
    else if ((trim_index_min >= size) || (trim_angle_max < angle_min))
    {
      // trim angle range is not in sensor range
      trim_index_min = trim_index_max = 0;
    }
  }
}

inline std::string getCameraTopic(const std::string& sensor)
{
  std::map<std::string, std::string> sensor_topic =
  {
    {"camera1", "/camera1"},
    {"camera2", "/camera2"},
    {"camera3", "/camera3"},
    {"camera4", "/camera4"},
    {"camera5", "/camera5"}
  };
  ros::param::get(std::string("/marker_localization/camera/") + sensor, sensor_topic[sensor]);
  return sensor_topic[sensor];
}

inline std::string getLaserTopic(const std::string& sensor)
{
  std::map<std::string, std::string> sensor_topic =
  {
    {"laser1", "/scan"},
    {"laser2", "/scan2"},
    {"laser3", "/scan3"},
    {"laser4", "/scan4"},
    {"laser5", "/scan5"},
    {"laser6", "/scan6"},
    {"camera1", "/camera1/scan"},
    {"camera2", "/camera2/scan"},
    {"camera3", "/camera3/scan"},
    {"camera4", "/camera4/scan"},
    {"camera5", "/camera5/scan"}
  };
  ros::param::get(std::string("/marker_localization/laser/") + sensor, sensor_topic[sensor]);
  return sensor_topic[sensor];
}

}  // namespace marker_localization


namespace tf2
{

template <>
inline
const ros::Time& getTimestamp(const geometry_msgs::PoseArray& t)  {return t.header.stamp;}

template <>
inline
const std::string& getFrameId(const geometry_msgs::PoseArray& t)  {return t.header.frame_id;}

template <>
inline
void doTransform(const geometry_msgs::PoseArray& t_in, geometry_msgs::PoseArray& t_out, const geometry_msgs::TransformStamped& transform)
{
  if (t_out.poses.size() != t_in.poses.size())
  {
    t_out.poses.resize(t_in.poses.size());
  }
  for (size_t i = 0; i < t_in.poses.size(); ++i)
  {
    doTransform(t_in.poses[i], t_out.poses[i], transform);
  }
  t_out.header.seq = t_in.header.seq;
  t_out.header.stamp = transform.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
}

template <>
inline
const ros::Time& getTimestamp(const sensor_msgs::LaserScan& t)  {return t.header.stamp;}

template <>
inline
const std::string& getFrameId(const sensor_msgs::LaserScan& t)  {return t.header.frame_id;}

inline
void doTransform(const std::complex<double>& t_in, float* t_out, const geometry_msgs::TransformStamped& transform)
{
  geometry_msgs::Point p_in, p_out;
  p_in.x = t_in.real();
  p_in.y = t_in.imag();
  p_in.z = 0.0;
  doTransform(p_in, p_out, transform);

  t_out[0] = p_out.x;
  t_out[1] = p_out.y;
  t_out[2] = p_out.z;
}

inline
void doTransform(const sensor_msgs::LaserScan& t_in, sensor_msgs::PointCloud2& t_out,
                 const geometry_msgs::TransformStamped& transform, const std::set<size_t>& indices = {})
{
  t_out.header.seq = t_in.header.seq;
  t_out.header.stamp = t_in.header.stamp;
  t_out.header.frame_id = transform.header.frame_id;
  t_out.height = 1;
  t_out.width = (indices.size() > 0) ? indices.size() : t_in.ranges.size();
  t_out.fields.resize(4);
  t_out.point_step = 0;
  t_out.fields[0].name = "x";
  t_out.fields[0].offset = t_out.point_step;
  t_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  t_out.fields[0].count = 1;
  t_out.point_step += (sizeof(float) * t_out.fields[0].count);
  t_out.fields[1].name = "y";
  t_out.fields[1].offset = t_out.point_step;
  t_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  t_out.fields[1].count = 1;
  t_out.point_step += (sizeof(float) * t_out.fields[1].count);
  t_out.fields[2].name = "z";
  t_out.fields[2].offset = t_out.point_step;
  t_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  t_out.fields[2].count = 1;
  t_out.point_step += (sizeof(float) * t_out.fields[2].count);
  t_out.fields[3].name = "index";
  t_out.fields[3].datatype = sensor_msgs::PointField::INT32;
  t_out.fields[3].offset = t_out.point_step;
  t_out.fields[3].count = 1;
  t_out.point_step += (sizeof(int32_t) * t_out.fields[3].count);
  t_out.is_bigendian = false;
  t_out.row_step = t_out.point_step * t_out.width;
  t_out.data.resize(t_out.row_step * t_out.height);
  t_out.is_dense = false;

  size_t count = 0;
  if (indices.size() > 0)
  {
    for (auto& i : indices)
    {
      if (i < t_in.ranges.size())
      {
        doTransform(std::polar<double>(t_in.ranges[i], t_in.angle_min + t_in.angle_increment * i),
                    reinterpret_cast<float*>(&t_out.data[count * t_out.point_step]), transform);
        *reinterpret_cast<int32_t*>(&t_out.data[count * t_out.point_step + t_out.fields[3].offset]) = i;

        // make sure to increment count
        ++count;
      }
    }
  }
  else
  {
    for (size_t i = 0; i < t_in.ranges.size(); i++)
    {
      float range = t_in.ranges[i];
      if (t_in.range_min <= range && range < t_in.range_max)
      {
        doTransform(std::polar<double>(range, t_in.angle_min + t_in.angle_increment * i),
                    reinterpret_cast<float*>(&t_out.data[count * t_out.point_step]), transform);
        *reinterpret_cast<int32_t*>(&t_out.data[count * t_out.point_step + t_out.fields[3].offset]) = i;

        // make sure to increment count
        ++count;
      }
    }
  }

  // resize if necessary
  if (t_out.width > count)
  {
    t_out.width = count;
    t_out.row_step = t_out.point_step * t_out.width;
    t_out.data.resize(t_out.row_step * t_out.height);
  }
}

inline
geometry_msgs::Pose2D& toMsg(const tf2::Transform& in, geometry_msgs::Pose2D& out)
{
  const tf2::Vector3& o = in.getOrigin();
  out.x = o.getX();
  out.y = o.getY();
  out.theta = tf2::getYaw(in.getRotation());
  return out;
}

inline
void fromMsg(const geometry_msgs::Pose2D& in, tf2::Transform& out)
{
  out.setOrigin(tf2::Vector3(in.x, in.y, 0.0));
  out.setRotation(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), in.theta));
}

}  // namespace tf2


#endif  // MARKER_LOCALIZATION_UTILS_H
