/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include <costmap_2d/observation_buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;
using namespace tf2;

namespace costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, tf2_ros::Buffer& tf2_buffer, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf2_buffer_(tf2_buffer), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  geometry_msgs::TransformStamped transformStamped;
  if (!tf2_buffer_.canTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<ObservationBuffer::ObservationScan>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      ObservationBuffer::ObservationScan& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf2_buffer_.transform(origin, origin, new_global_frame);
      obs.origin_ = origin.point;

      if (obs.scan_)
      {
        tf2::Transform t;
        geometry_msgs::TransformStamped transform;

        transform.header.frame_id = global_frame_;
        transform.header.stamp = transform_time;
        transform.child_frame_id = obs.cloud_->header.frame_id;

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        transform = tf2_buffer_.transform(transform, new_global_frame);
        fromMsg(transform.transform, t);
        obs.transform_ = t * obs.transform_;
        obs.cloud_->header.frame_id = new_global_frame;
      }
      else
      {
        // we also need to transform the cloud of the observation to the new global frame
        tf2_buffer_.transform(*(obs.cloud_), *(obs.cloud_), new_global_frame);
      }
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  geometry_msgs::PointStamped global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(ObservationBuffer::ObservationScan());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;
    observation_list_.front().raytrace_split_ = 0.0;

    sensor_msgs::PointCloud2& global_frame_cloud = *(observation_list_.front().cloud_);

    // transform the point cloud
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

#if 0  // below or above height thresholds is valid for raytrace freespace
    // now we need to remove observations from the cloud that are below or above our height thresholds
    sensor_msgs::PointCloud2& observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step)
    {
      if ((*iter_z) <= max_obstacle_height_
          && (*iter_z) >= min_obstacle_height_)
      {
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // resize the cloud for the number of legal points
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
#else
    observation_list_.front().min_obstacle_height_ = min_obstacle_height_;
    observation_list_.front().max_obstacle_height_ = max_obstacle_height_;
#endif
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

void ObservationBuffer::bufferCloud(const sensor_msgs::LaserScan& scan, double raytrace_split)
{
  tf2::Transform t;
  geometry_msgs::TransformStamped transform;

  transform.header.frame_id = sensor_frame_ == "" ? scan.header.frame_id : sensor_frame_;
  transform.header.stamp = scan.header.stamp;
  transform.child_frame_id = transform.header.frame_id;

  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  // Transform to global frame
  try
  {
    transform = tf2_buffer_.transform(transform, global_frame_);
    fromMsg(transform.transform, t);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("TF Exception for sensor frame: %s, global frame: %s, %s", transform.child_frame_id.c_str(),
              global_frame_.c_str(), ex.what());
    return;
  }

  observation_list_.push_front(ObservationBuffer::ObservationScan(scan));
  ObservationBuffer::ObservationScan& observation = observation_list_.front();

  observation.transform_ = t;

  observation.origin_.x = transform.transform.translation.x;
  observation.origin_.y = transform.transform.translation.y;
  observation.origin_.z = transform.transform.translation.z;

  // make sure to pass on the parameters of the observation buffer to the observations
  observation.raytrace_range_ = raytrace_range_;
  observation.obstacle_range_ = obstacle_range_;
  observation.raytrace_split_ = (raytrace_split < raytrace_range_) ? raytrace_split : 0.0;
  observation.min_obstacle_height_ = min_obstacle_height_;
  observation.max_obstacle_height_ = max_obstacle_height_;

  sensor_msgs::PointCloud2& observation_cloud = *(observation.cloud_);

  observation_cloud.header.seq = scan.header.seq;
  observation_cloud.header.stamp = scan.header.stamp;
  observation_cloud.header.frame_id = global_frame_;

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<ObservationBuffer::ObservationScan>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    if (obs_it->scan_)
    {
      const sensor_msgs::LaserScan& scan = *(obs_it->scan_);
      sensor_msgs::PointCloud2& cloud = *(obs_it->cloud_);

      cloud.height = 1;
      cloud.width = scan.ranges.size();
      cloud.fields.resize(5);
      cloud.is_bigendian = false;
      cloud.point_step = 0;
      cloud.fields[0].name = "x";
      cloud.fields[0].offset = cloud.point_step;
      cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[0].count = 1;
      cloud.point_step += (sizeof(float) * cloud.fields[0].count);
      cloud.fields[1].name = "y";
      cloud.fields[1].offset = cloud.point_step;
      cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[1].count = 1;
      cloud.point_step += (sizeof(float) * cloud.fields[1].count);
      cloud.fields[2].name = "z";
      cloud.fields[2].offset = cloud.point_step;
      cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[2].count = 1;
      cloud.point_step += (sizeof(float) * cloud.fields[2].count);
      cloud.fields[3].name = "distances";
      cloud.fields[3].offset = cloud.point_step;
      cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[3].count = 1;
      cloud.point_step += (sizeof(float) * cloud.fields[3].count);
      cloud.fields[4].name = "index";
      cloud.fields[4].datatype = sensor_msgs::PointField::INT32;
      cloud.fields[4].offset = cloud.point_step;
      cloud.fields[4].count = 1;
      cloud.point_step += (sizeof(int32_t) * cloud.fields[4].count);
      cloud.row_step = 0;  // cloud.point_step * cloud.width
      cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
      cloud.is_dense = false;

      size_t count = 0;
      float angle = scan.angle_min;
      for (size_t i = 0; i < cloud.width; i++, angle += scan.angle_increment)
      {
        const float range = scan.ranges[i];
        if (scan.range_min <= range && range < scan.range_max)
        {
          tf2::Vector3 v = obs_it->transform_ * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
          float *pstep = reinterpret_cast<float*>(&cloud.data[cloud.row_step]);
          pstep[0] = v.getX();
          pstep[1] = v.getY();
          pstep[2] = v.getZ();
          pstep[3] = range;
          *reinterpret_cast<int32_t*>(&pstep[4]) = i;

          cloud.row_step += cloud.point_step;
          ++count;
        }
      }

      // resize if necessary
      if (cloud.width > count)
      {
        cloud.width = count;
        cloud.data.resize(cloud.row_step * cloud.height);
      }

      obs_it->scan_.reset();
    }

    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<ObservationBuffer::ObservationScan>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      ObservationBuffer::ObservationScan& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace costmap_2d

