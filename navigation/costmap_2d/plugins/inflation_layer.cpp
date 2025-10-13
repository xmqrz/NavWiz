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
 *         David V. Lu!!
 *********************************************************************/
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : resolution_(0)
  , inflation_radius_(0)
  , inscribed_radius_(0)
  , weight_(0)
  , inflate_unknown_(false)
  , footprint_clearing_enabled_(false)
  , footprint_lethal_cost_(INSCRIBED_INFLATED_OBSTACLE)
  , cell_inflation_radius_(0)
  , cell_inflation_radius_sq_(0)
  , cell_inscribed_radius_(0)
  , cell_inscribed_radius_sq_(0)
  , dsrv_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb =
      [this](auto& config, auto level) { reconfigureCB(config, level); };

    if (dsrv_ != NULL)
    {
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }

  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown)
  {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }

  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  footprint_lethal_cost_ = config.footprint_lethal_cost;
}

void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  seen_.resize(size_x * size_y);
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y)
{
  if (need_reinflation_)
  {
    computeCaches();
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    if (footprint_clearing_enabled_)
    {
      *min_x = std::min(robot_x - inflation_radius_, *min_x);
      *min_y = std::min(robot_y - inflation_radius_, *min_y);
      *max_x = std::max(robot_x + inflation_radius_, *max_x);
      *max_y = std::max(robot_y + inflation_radius_, *max_y);
    }
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }

  robot_x_ = robot_x;
  robot_y_ = robot_y;
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inscribed_radius_ = cellDistance(inscribed_radius_);
  cell_inscribed_radius_sq_ = cell_inscribed_radius_ * cell_inscribed_radius_;
  need_reinflation_ = true;

  ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (cell_inflation_radius_ == 0)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_.size() != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    seen_.resize(size_x * size_y);
  }

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // Inscribed list; we append cells to visit in a list associated with its distance to the robot
  std::vector<CellData> obs_inscribed;
  unsigned int rx, ry, sq_dist = 0;
  if (footprint_clearing_enabled_ && (footprint_lethal_cost_ > 1))
  {
    if (master_grid.worldToMap(robot_x_, robot_y_, rx, ry))
    {
      sq_dist = cell_inscribed_radius_sq_ << 2;  // (cell_inscribed_radius_ * 2) ^ 2
    }
  }

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  // std::vector<CellData>& obs_bin = inflation_cells_[0];
  int row_index = master_grid.getIndex(min_i, min_j);
  for (int j = min_j; j < max_j; ++j, row_index += size_x)
  {
    int index = row_index;
    for (int i = min_i; i < max_i; ++i, ++index)
    {
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        // obs_bin.push_back(CellData(index, i, j, 0, 0));
        if (sq_dist)
        {
          int dx = i - rx;
          int dy = j - ry;
          if ((dx * dx + dy * dy) < sq_dist)
          {
            obs_inscribed.push_back(CellData(index, i, j, dx, dy));
          }
        }

        seen_[index] = true;

        // attempt to put the neighbors of the current cell onto the inflation list
        if (i > min_i)
          enqueue(index - 1, i - 1, j, -1, 0, 1);
        if (j > min_j)
          enqueue(index - size_x, i, j - 1, 0, -1, 1);

        unsigned int idx;
        if (i < max_i - 1)
        {
          if (master_array[idx = index + 1] != LETHAL_OBSTACLE)
          {
            seen_[idx] = false;
            enqueue(idx, i + 1, j, 1, 0, 1);
          }
        }
        if (j < max_j - 1)
        {
          if (master_array[idx = index + size_x] != LETHAL_OBSTACLE)
          {
            seen_[idx] = false;
            enqueue(idx, i, j + 1, 0, 1, 1);
          }
        }
      }
      else
      {
        seen_[index] = false;
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (sq_dist = 1; sq_dist <= cell_inflation_radius_sq_; ++sq_dist)
  {
    unsigned char cost = cached_costs_[sq_dist];
    if (!cost)
    {
      continue;
    }

    std::vector<CellData>& bin = inflation_cells_[sq_dist];
    for (int i = 0; i < bin.size(); ++i)
    {
      const CellData& cell = bin[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      int dx = cell.dx_;
      int dy = cell.dy_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char old_cost = master_array[index];
      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        master_array[index] = cost;
      else
        master_array[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the inflation list
      if (dx <= 0 && mx > min_i)
        enqueue(index - 1, mx - 1, my, dx - 1, dy, sq_dist - dx - dx + 1);
      if (dy <= 0 && my > min_j)
        enqueue(index - size_x, mx, my - 1, dx, dy - 1, sq_dist - dy - dy + 1);
      if (dx >= 0 && mx < max_i - 1)
        enqueue(index + 1, mx + 1, my, dx + 1, dy, sq_dist + dx + dx + 1);
      if (dy >= 0 && my < max_j - 1)
        enqueue(index + size_x, mx, my + 1, dx, dy + 1, sq_dist + dy + dy + 1);
    }
    bin.clear();
  }

  if (obs_inscribed.size())
  {
    min_i = rx - cell_inscribed_radius_;
    min_j = ry - cell_inscribed_radius_;
    max_i = rx + cell_inscribed_radius_ + 1;
    max_j = ry + cell_inscribed_radius_ + 1;

    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(int(size_x), max_i);
    max_j = std::min(int(size_y), max_j);

    row_index = master_grid.getIndex(min_i, min_j);
    for (int j = min_j, dj = j - ry; j < max_j; ++j, ++dj, row_index += size_x)
    {
      int index = row_index;
      for (int i = min_i, di = i - rx; i < max_i; ++i, ++di, ++index)
      {
        unsigned char cost = master_array[index];
        if ((cost < LETHAL_OBSTACLE) && (cost > (footprint_lethal_cost_ - 2)))
        {
          if ((di * di + dj * dj) < cell_inscribed_radius_sq_)
          {
            bool clear = true;
            for (int k = 0; k < obs_inscribed.size(); k++)
            {
              const CellData& cell = obs_inscribed[k];
              int dx = cell.x_ - i;
              int dy = cell.y_ - j;
              if ((dx * dx + dy * dy) < cell_inscribed_radius_sq_)
              {
                if ((di * cell.dx_ + dj * cell.dy_) > 0)  // dot product
                {
                  clear = false;
                  break;
                }
              }
            }
            if (clear)
            {
              master_array[index] = footprint_lethal_cost_ - 2;
            }
          }
        }
      }
    }
  }
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  dx The x offset to the obstacle point inflation started at
 * @param  dy The y offset to the obstacle point inflation started at
 * @param  sq_dist The squared distance to the obstacle point inflation started at (= dx * dx + dy * dy)
 */
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    int dx, int dy, unsigned int sq_dist)
{
  if (!seen_[index])
  {
    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (sq_dist > cell_inflation_radius_sq_)
      return;

    if (cached_costs_[sq_dist] == FREE_SPACE)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[sq_dist].push_back(CellData(index, mx, my, dx, dy));
  }
}

void InflationLayer::computeCaches()
{
  if (cell_inflation_radius_ == 0)
    return;

  cell_inflation_radius_sq_ = cell_inflation_radius_ * cell_inflation_radius_;
  inflation_cells_.resize(cell_inflation_radius_sq_ + 1);
  cached_costs_.resize(cell_inflation_radius_sq_ + 1);

  for (unsigned int i = 0, sq_dist0 = 0; i <= cell_inflation_radius_; sq_dist0 += ++i + i - 1)
  {
    for (unsigned int j = 0, sq_dist = sq_dist0; j <= i && sq_dist <= cell_inflation_radius_sq_; sq_dist += ++j + j - 1)
    {
      cached_costs_[sq_dist] = computeCost(sqrt(sq_dist));
    }
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
  }
}

}  // namespace costmap_2d
