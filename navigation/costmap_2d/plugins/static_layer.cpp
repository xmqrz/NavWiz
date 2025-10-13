/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PNG decoding
#include <png++/png.hpp>
#include <strstream>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
    [this](auto& config, auto level){ reconfigureCB(config, level); };
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

/**
 * Decode the png data in the OccupancyGrid map message.
 */
class pixel_consumer: public png::consumer<png::index_pixel, pixel_consumer>
{
public:
  pixel_consumer(png::image_info& info, std::vector<int8_t>& pixels) :
    png::consumer<png::index_pixel, pixel_consumer>(info),
    pixels_(pixels)
  {}

  void reset(size_t pass)
  {
    if (pass == 0)
    {
      pixels_.resize(get_info().get_height() * get_info().get_width());
    }
  }

  png::byte* get_next_row(size_t pos)
  {
    return reinterpret_cast<png::byte*>(&pixels_[pos * get_info().get_width()]);
  }

protected:
  std::vector<int8_t>& pixels_;
};

void StaticLayer::decodePng(std::vector<int8_t>& data)
{
  const int8_t PNG_MAGIC_BYTES[8] = {-0x77, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  if (data.size() > 8 &&
      std::equal(data.begin(), data.begin() + 8, PNG_MAGIC_BYTES))
  {
    std::istrstream buf(reinterpret_cast<char*>(data.data()), data.size());

    png::image_info info;
    std::vector<int8_t> pixels;

    try
    {
      pixel_consumer consumer(info, pixels);
      consumer.read(buf, png::require_color_space<png::index_pixel>());
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("Error decoding png: " << ex.what());
      return;
    }

    data.swap(pixels);
  }
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  // Skip if still the same map. Map should have unique timestamp
  if (new_map->header.stamp == map_stamp_)
  {
    return;
  }

  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  ros::NodeHandle private_nh("~");
  unsigned int step = 1;
  double resolution = new_map->info.resolution;
  for (size_t i = name_.find('/'); i != std::string::npos; i = name_.find('/', i + 1))
  {
    double config_resolution;
    if (private_nh.getParam(name_.substr(0, i + 1) + "resolution", config_resolution))
    {
      if (config_resolution > resolution)
      {
        step = config_resolution / resolution;
        if (step > 1)
        {
          size_x += (step - 1);
          size_x /= step;
          size_y += (step - 1);
          size_y /= step;
          resolution *= step;
        }
      }
      map_stamp_ = new_map->header.stamp;
      break;
    }
  }

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, resolution);
    layered_costmap_->resizeMap(size_x, size_y, resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true /* set size_locked to true, prevents reconfigureCb from overriding map size*/);
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, resolution);
    resizeMap(size_x, size_y, resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  decodePng(const_cast<std::vector<int8_t>&>(new_map->data));

  unsigned int index = 0;
  unsigned int height = new_map->info.height;
  unsigned int width = new_map->info.width;

  // initialize the costmap with static data
  for (unsigned int h = 0; h < height; h += step)
  {
    for (unsigned int w = 0; w < width; w += step)
    {
      if (step > 1)
      {
        unsigned char cost = 0;
        for (unsigned int i = 0, y = h, it = h * width; i < step && y < height; ++i, ++y, it += width)
        {
          for (unsigned int j = 0, x = w, idx = it + w; j < step && x < width; ++j, ++x, ++idx)
          {
            unsigned char value = new_map->data[idx];
            value = interpretValue(value) + 1;
            cost = std::max(cost, value);
          }
        }
        costmap_[index] = cost - 1;
      }
      else
      {
        unsigned char value = new_map->data[index];
        costmap_[index] = interpretValue(value);
      }
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  decodePng(const_cast<std::vector<int8_t>&>(update->data));

  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  // onInitialize();
}

void StaticLayer::deactivate()
{
  if (map_stamp_.isZero())
  {
    ros::NodeHandle g_nh;

    map_sub_.shutdown();
    map_sub_ = g_nh.subscribe(map_sub_.getTopic(), 1, &StaticLayer::incomingMap, this);
  }
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{

  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
