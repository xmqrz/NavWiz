/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#include <pluginlib/class_list_macros.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <math.h>

#include "agv05_navx/costmap_layers.h"

PLUGINLIB_EXPORT_CLASS(agv05::ForbiddenLayer, costmap_2d::Layer)

typedef CGAL::Cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;


namespace agv05
{

ForbiddenLayer::ForbiddenLayer()
{
  zone_.polygons.resize(0);
}

ForbiddenLayer::~ForbiddenLayer()
{
  zone_.polygons.resize(0);
}

void ForbiddenLayer::onInitialize(void)
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  bool invert;
  nh.param("invert", invert, false);
  default_value_ = invert ? costmap_2d::LETHAL_OBSTACLE : costmap_2d::NO_INFORMATION;
  matchSize();

  // get the topics that we'll subscribe to from the parameter server
  std::string topic_string;
  nh.param("zone_topic", topic_string, std::string("forbidden_zone"));
  sub_ = nh.subscribe<agv05_msgs::PolygonArray>(topic_string, 1, &ForbiddenLayer::zoneCB, this);

  dsrv_ = boost::make_shared< dynamic_reconfigure::Server<agv05_navx::ForbiddenLayerConfig> >(nh);
  dsrv_->setCallback(boost::bind(&ForbiddenLayer::reconfigureCB, this, _1, _2));
}

void ForbiddenLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void ForbiddenLayer::updateMap(void)
{
  unsigned char cost = default_value_ == costmap_2d::NO_INFORMATION ? costmap_2d::LETHAL_OBSTACLE : costmap_2d::NO_INFORMATION;
  bounds_.min_x = std::numeric_limits<float>::max();
  bounds_.min_y = std::numeric_limits<float>::max();
  bounds_.max_x = -std::numeric_limits<float>::max();
  bounds_.max_y = -std::numeric_limits<float>::max();

  matchSize();

  for (std::size_t i = 0; i < zone_.polygons.size(); i++)
  {
    unsigned int mx, my;
    double wx, wy;

    switch (zone_.polygons[i].points.size())
    {
    case 0:
      break;
    case 1:
    {
      touch(zone_.polygons[i].points[0].x, zone_.polygons[i].points[0].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
      if (worldToMap(zone_.polygons[i].points[0].x, zone_.polygons[i].points[0].y, mx, my))
      {
        setCost(mx, my, cost);
      }
      break;
    }
    case 2:
    {
      touch(zone_.polygons[i].points[0].x, zone_.polygons[i].points[0].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
      touch(zone_.polygons[i].points[1].x, zone_.polygons[i].points[1].y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);

      drawLineSegment(zone_.polygons[i].points[0].x,
                      zone_.polygons[i].points[0].y,
                      zone_.polygons[i].points[1].x,
                      zone_.polygons[i].points[1].y,
                      cost);
      break;
    }
    default:
    {
      int min_i, min_j, max_i, max_j;
      double min_x = std::numeric_limits<float>::max();
      double min_y = std::numeric_limits<float>::max();
      double max_x = -std::numeric_limits<float>::max();
      double max_y = -std::numeric_limits<float>::max();
      Point_2 points[zone_.polygons[i].points.size()];

      for (std::size_t j = 0; j < zone_.polygons[i].points.size(); j++)
      {
        touch(zone_.polygons[i].points[j].x, zone_.polygons[i].points[j].y, &min_x, &min_y, &max_x, &max_y);
        points[j] = Point_2(zone_.polygons[i].points[j].x, zone_.polygons[i].points[j].y);

        drawLineSegment(zone_.polygons[i].points[j ? j - 1 : zone_.polygons[i].points.size() - 1].x,
                        zone_.polygons[i].points[j ? j - 1 : zone_.polygons[i].points.size() - 1].y,
                        zone_.polygons[i].points[j].x,
                        zone_.polygons[i].points[j].y,
                        cost);
      }
      touch(min_x, min_y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);
      touch(max_x, max_y, &bounds_.min_x, &bounds_.min_y, &bounds_.max_x, &bounds_.max_y);

      worldToMapEnforceBounds(min_x, min_y, min_i, min_j);
      worldToMapEnforceBounds(max_x, max_y, max_i, max_j);

      for (mx = min_i; mx < max_i; mx++)
      {
        for (my = min_j; my < max_j; my++)
        {
          mapToWorld(mx, my, wx, wy);
          switch (CGAL::bounded_side_2(points, points + zone_.polygons[i].points.size(), Point_2(wx, wy), Kernel()))
          {
          case CGAL::ON_BOUNDED_SIDE:
          case CGAL::ON_BOUNDARY:
            setCost(mx, my, cost);
            break;
          case CGAL::ON_UNBOUNDED_SIDE:
          default:
            break;
          }
        }
      }
      break;
    }
    }
  }
}

void ForbiddenLayer::zoneCB(const agv05_msgs::PolygonArray::ConstPtr& msg)
{
  ROS_INFO("agv05_navx: forbidden layer new zone");

  zone_ = *msg;
  update_ = true;
}

void ForbiddenLayer::reconfigureCB(agv05_navx::ForbiddenLayerConfig &config, uint32_t level)
{
  ROS_INFO("agv05_navx: forbidden layer config received");

  if (enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    update_ = true;
  }

  unsigned char cost = config.invert ? costmap_2d::LETHAL_OBSTACLE : costmap_2d::NO_INFORMATION;
  if (default_value_ != cost)
  {
    default_value_ = cost;
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
