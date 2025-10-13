/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <pluginlib/class_list_macros.h>
#include <CGAL/Cartesian.h>
#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Arr_Bezier_curve_traits_2.h>

#include <math.h>

#include "agv05_navx/costmap_layers.h"

PLUGINLIB_EXPORT_CLASS(agv05::StaticPathLayer, costmap_2d::Layer)

typedef CGAL::CORE_algebraic_number_traits Nt_traits;
typedef Nt_traits::Rational Rational;
typedef CGAL::Cartesian<Rational> Rat_kernel;
typedef CGAL::Cartesian<Nt_traits::Algebraic> Alg_kernel;
typedef CGAL::Arr_Bezier_curve_traits_2<Rat_kernel, Alg_kernel, Nt_traits>::Curve_2 Bezier_2;
typedef Rat_kernel::Point_2 Point_2;

#define _DOUBLE(d) CGAL::to_double(d)
#define _SQRT(sq)  std::sqrt(_DOUBLE(sq))


namespace agv05
{

StaticPathLayer::StaticPathLayer()
{
  cache_.clear();
  path_.paths.resize(0);
  path_.area.points.resize(0);
  area_.bound = true;
  area_.min_i = area_.min_j = area_.max_i = area_.max_j = 0;
}

StaticPathLayer::~StaticPathLayer()
{
  cache_.clear();
  path_.paths.resize(0);
  path_.area.points.resize(0);
}

void StaticPathLayer::onInitialize(void)
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  int cost;
  nh.param("cost", cost, costmap_2d::FREE_SPACE + 8);
  if (cost < costmap_2d::FREE_SPACE)
  {
    default_value_ = costmap_2d::FREE_SPACE;
  }
  else if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    default_value_ = cost;
  }
  else
  {
    default_value_ = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
  }
  matchSize();

  nh.param("gradient_distance", gradient_dist_, 0.0);
  if (gradient_dist_ < 0.0)
  {
    gradient_dist_ = 0.0;
  }
  computeCaches();

  nh.param("border_cost", area_.border_cost, static_cast<int>(costmap_2d::NO_INFORMATION));

  // get the topics that we'll subscribe to from the parameter server
  std::string topic_string;
  nh.param("path_topic", topic_string, std::string("static_path"));
  sub_ = nh.subscribe<agv05_msgs::Path>(topic_string, 1, &StaticPathLayer::pathCB, this);

  dsrv_ = boost::make_shared< dynamic_reconfigure::Server<agv05_navx::StaticPathLayerConfig> >(nh);
  dsrv_->setCallback(boost::bind(&StaticPathLayer::reconfigureCB, this, _1, _2));
}

void StaticPathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  if (area_.bound)
  {
    min_i = std::max(min_i, area_.min_i);
    min_j = std::max(min_j, area_.min_j);
    max_i = std::min(max_i, area_.max_i);
    max_j = std::min(max_j, area_.max_j);
  }

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++, it++)
    {
      if (master_array[it] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        int sum = master_array[it] + costmap_[it];
        if (sum < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          master_array[it] = sum;
        }
        else if (costmap_[it] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          master_array[it] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        }
        else
        {
          master_array[it] = costmap_[it];
        }
      }
    }
  }
}

void StaticPathLayer::deactivate()
{
  bounds_.min_x = -std::numeric_limits<float>::max();
  bounds_.min_y = -std::numeric_limits<float>::max();
  bounds_.max_x = std::numeric_limits<float>::max();
  bounds_.max_y = std::numeric_limits<float>::max();

  path_.paths.resize(0);
  path_.area.points.resize(0);
  area_.bound = true;
  area_.min_i = area_.min_j = area_.max_i = area_.max_j = 0;
}

unsigned char StaticPathLayer::gradientCost(double dist)
{
  unsigned char cost = costmap_2d::FREE_SPACE;

  if (dist < getResolution())
  {
    return cost;
  }
  else if (dist < gradient_dist_)
  {
    cost += (default_value_ - cost) * dist / gradient_dist_;
    return cost;
  }

  return default_value_;
}

void StaticPathLayer::computeCaches()
{
  unsigned int cell_dist = cellDistance(gradient_dist_);
  unsigned int cell_dist_sq = cell_dist * cell_dist;

  cache_.resize(cell_dist_sq);
  cache_[0] = StaticPathLayer::Cache(gradientCost(0.0));

  for (std::size_t i = 1; i < cell_dist; i++)
  {
    for (std::size_t j = 0; j <= i; j++)
    {
      unsigned int dist_sq = i * i + j * j;
      if (dist_sq < cell_dist_sq)
      {
        cache_[dist_sq] = StaticPathLayer::Cache(gradientCost(std::sqrt(dist_sq) * getResolution()));
      }
    }
  }

  cache_update_ = false;
}

void StaticPathLayer::enqueue(double x, double y)
{
  unsigned int mx, my;
  if (worldToMap(x, y, mx, my))
  {
    cache_[0].cells_.push_back(CellData(getIndex(mx, my), mx, my, mx, my));
  }
}

void StaticPathLayer::enqueue(std::vector<bool>& seen, unsigned int index,
                              unsigned int mx, unsigned int my,
                              unsigned int src_x, unsigned int src_y)
{
  if (!seen[index])
  {
    int dx = mx - src_x;
    int dy = my - src_y;
    unsigned int dist_sq = dx * dx + dy * dy;

    if (dist_sq < cache_.size())
    {
      cache_[dist_sq].cells_.push_back(CellData(index, mx, my, src_x, src_y));
    }
  }
}

void StaticPathLayer::dequeue()
{
  unsigned int size_x = getSizeInCellsX();
  unsigned int size_y = getSizeInCellsY();
  unsigned int max_i = size_x - 1;
  unsigned int max_j = size_y - 1;
  std::vector<bool> seen(size_x * size_y, false);

  for (auto& cache : cache_)
  {
    for (const auto& cell : cache.cells_)
    {
      if (!seen[cell.index_])
      {
        costmap_[cell.index_] = cache.cost_;
        seen[cell.index_] = true;

        // attempt to put the neighbors of the current cell onto the list
        if (cell.x_ > 0)
          enqueue(seen, cell.index_ - 1, cell.x_ - 1, cell.y_, cell.src_x_, cell.src_y_);
        if (cell.y_ > 0)
          enqueue(seen, cell.index_ - size_x, cell.x_, cell.y_ - 1, cell.src_x_, cell.src_y_);
        if (cell.x_ < max_i)
          enqueue(seen, cell.index_ + 1, cell.x_ + 1, cell.y_, cell.src_x_, cell.src_y_);
        if (cell.y_ < max_j)
          enqueue(seen, cell.index_ + size_x, cell.x_, cell.y_ + 1, cell.src_x_, cell.src_y_);
      }
    }
    cache.cells_.clear();
  }
}

void StaticPathLayer::updateMap(void)
{
  bool path_empty = true;
  bool area_bound = true;

  if (path_.area.points.size() < 3)
  {
    path_.area.points.resize(0);
    area_bound = false;
  }

  bounds_.min_x = std::numeric_limits<float>::max();
  bounds_.min_y = std::numeric_limits<float>::max();
  bounds_.max_x = -std::numeric_limits<float>::max();
  bounds_.max_y = -std::numeric_limits<float>::max();

  matchSize();

  if (cache_update_)
  {
    computeCaches();
  }

  for (std::size_t i = 0; i < path_.paths.size(); i++)
  {
    switch (path_.paths[i].points.size())
    {
    case 0:
      break;
    case 1:
    {
      path_empty = false;
      if (!area_bound)
      {
        touch(path_.paths[i].points[0].x, path_.paths[i].points[0].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
      }
      enqueue(path_.paths[i].points[0].x, path_.paths[i].points[0].y);
      break;
    }
    default:
    {
      Point_2 points[path_.paths[i].points.size()];
      double dist = 0;

      path_empty = false;
      for (std::size_t j = 0; j < path_.paths[i].points.size(); j++)
      {
        if (!area_bound)
        {
          touch(path_.paths[i].points[j].x, path_.paths[i].points[j].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
        }
        points[j] = Point_2(path_.paths[i].points[j].x, path_.paths[i].points[j].y);
        if (j)
        {
          dist += _SQRT(CGAL::squared_distance(points[j - 1], points[j]));
        }
      }

      Bezier_2 bezier(points, points + path_.paths[i].points.size());
      std::size_t steps = cellDistance(dist);

      if (steps)
      {
        Rational dt = (1.0 / steps);
        Rational t = 0;

        for (std::size_t j = 0; j < steps; j++, t += dt)
        {
          Point_2 p = bezier(t);
          enqueue(_DOUBLE(p.x()), _DOUBLE(p.y()));
        }
      }
      enqueue(path_.paths[i].points[path_.paths[i].points.size() - 1].x, path_.paths[i].points[path_.paths[i].points.size() - 1].y);
      break;
    }
    }
  }

  if (path_empty)
  {
    path_.paths.resize(0);
  }
  else
  {
    dequeue();
  }

  if (path_.area.points.size())
  {
    for (std::size_t i = 0; i < path_.area.points.size(); i++)
    {
      touch(path_.area.points[i].x, path_.area.points[i].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
      drawLineSegment(path_.area.points[i ? i - 1 : path_.area.points.size() - 1].x,
                      path_.area.points[i ? i - 1 : path_.area.points.size() - 1].y,
                      path_.area.points[i].x,
                      path_.area.points[i].y,
                      area_.border_cost);
    }

    bounds_.min_x -= resolution_;
    bounds_.min_y -= resolution_;
    bounds_.max_x += resolution_;
    bounds_.max_y += resolution_;
  }
  else if (!path_empty)
  {
    bounds_.min_x -= gradient_dist_;
    bounds_.min_y -= gradient_dist_;
    bounds_.max_x += gradient_dist_;
    bounds_.max_y += gradient_dist_;
  }

  area_bound |= path_empty;
  if (area_bound)
  {
    worldToMapEnforceBounds(bounds_.min_x, bounds_.min_y, area_.min_i, area_.min_j);
    worldToMapEnforceBounds(bounds_.max_x, bounds_.max_y, area_.max_i, area_.max_j);
  }
  if (area_.bound != area_bound)
  {
    area_.bound = area_bound;
    bounds_.min_x = -std::numeric_limits<float>::max();
    bounds_.min_y = -std::numeric_limits<float>::max();
    bounds_.max_x = std::numeric_limits<float>::max();
    bounds_.max_y = std::numeric_limits<float>::max();
  }
}

void StaticPathLayer::pathCB(const agv05_msgs::Path::ConstPtr& msg)
{
  ROS_INFO("agv05_navx: static path layer new path");

  path_ = *msg;
  update_ = true;
}

void StaticPathLayer::reconfigureCB(agv05_navx::StaticPathLayerConfig &config, uint32_t level)
{
  ROS_INFO("agv05_navx: static path layer config received");

  if (enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    update_ = true;
  }

  int cost = config.cost;
  if (cost < costmap_2d::FREE_SPACE)
  {
    cost = costmap_2d::FREE_SPACE;
  }
  else if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
  }
  if (default_value_ != cost)
  {
    default_value_ = cost;
    update_ = true;
    cache_update_ = true;
  }

  double gradient_dist = config.gradient_distance;
  if (gradient_dist < 0.0)
  {
    gradient_dist = 0.0;
  }
  if (gradient_dist_ != gradient_dist)
  {
    gradient_dist_ = gradient_dist;
    update_ = true;
    cache_update_ = true;
  }

  if (area_.border_cost != config.border_cost)
  {
    area_.border_cost = config.border_cost;
    update_ = true;
  }

  if (update_)
  {
    bounds_.min_x = -std::numeric_limits<float>::max();
    bounds_.min_y = -std::numeric_limits<float>::max();
    bounds_.max_x = std::numeric_limits<float>::max();
    bounds_.max_y = std::numeric_limits<float>::max();
  }
}

};  // namespace agv05
