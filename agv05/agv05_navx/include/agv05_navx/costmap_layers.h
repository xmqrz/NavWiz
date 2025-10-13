/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Wong Tze Lin
 */

#ifndef AGV05_NAVX_COSTMAP_LAYERS_H
#define AGV05_NAVX_COSTMAP_LAYERS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <boost/shared_ptr.hpp>

#include <agv05_msgs/Path.h>
#include <agv05_msgs/PolygonArray.h>
// #include <agv05_msgs/NavxActionActionGoal.h>

#include <agv05_navx/ForbiddenLayerConfig.h>
#include <agv05_navx/StaticPathLayerConfig.h>

#include <CGAL/Cartesian.h>

namespace agv05
{

class CellData
{
public:
  explicit CellData(unsigned int i = 0, unsigned int x = 0, unsigned int y = 0,
                    unsigned int sx = 0, unsigned int sy = 0) :
    index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

class AGV05StaticLayer : public costmap_2d::CostmapLayer
{
public:
  AGV05StaticLayer()  // extended initializer lists only available with -std=c++11 or -std=gnu++11
  {
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    bounds_.min_x = std::numeric_limits<float>::max();
    bounds_.min_y = std::numeric_limits<float>::max();
    bounds_.max_x = -std::numeric_limits<float>::max();
    bounds_.max_y = -std::numeric_limits<float>::max();

    update_ = false;
  }

  virtual ~AGV05StaticLayer() {}

  virtual void onInitialize(void) {}

  virtual void updateBounds(double robot_x,
                            double robot_y,
                            double robot_yaw,
                            double* min_x,
                            double* min_y,
                            double* max_x,
                            double* max_y)
  {
    // ros::spinOnce();
    if (update_)
    {
      unionBounds(min_x, min_y, max_x, max_y);
      updateMap();
      unionBounds(min_x, min_y, max_x, max_y);
      update_ = false;
    }
  }

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {}

protected:
  virtual void unionBounds(double* min_x, double* min_y, double* max_x, double* max_y)
  {
    *min_x = std::min(bounds_.min_x, *min_x);
    *min_y = std::min(bounds_.min_y, *min_y);
    *max_x = std::max(bounds_.max_x, *max_x);
    *max_y = std::max(bounds_.max_y, *max_y);
  }

  virtual void drawLineSegment(double x1, double y1, double x2, double y2, unsigned char cost)
  {
    unsigned int mx, my;
    CGAL::Cartesian<double>::Point_2 p(x1, y1), q(x2, y2);
    CGAL::Cartesian<double>::Segment_2 s(p, q);
    std::size_t size = cellDistance(std::sqrt(s.squared_length()));

    if (size)
    {
      CGAL::Cartesian<double>::Vector_2 v = s.to_vector() / size;
      for (std::size_t i = 0; i < size; i++)
      {
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          setCost(mx, my, cost);
        }
#if CGAL_VERSION_NR < 1041000000
        p = p + v;
#else
        p += v;
#endif
      }
    }
    if (worldToMap(q.x(), q.y(), mx, my))
    {
      setCost(mx, my, cost);
    }
  }

  virtual void updateMap(void) {}

  struct
  {
    double min_x, min_y, max_x, max_y;
  } bounds_;
  bool update_;
};

class ForbiddenLayer : public AGV05StaticLayer
{
public:
  ForbiddenLayer();
  virtual ~ForbiddenLayer();

  virtual void onInitialize(void);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  virtual void updateMap(void);

  /* ROS subscribers and messages */
  virtual void zoneCB(const agv05_msgs::PolygonArray::ConstPtr& msg);
  ros::Subscriber sub_;
  agv05_msgs::PolygonArray zone_;

  /* Dynamic reconfigure server */
  virtual void reconfigureCB(agv05_navx::ForbiddenLayerConfig &config, uint32_t level);
  boost::shared_ptr< dynamic_reconfigure::Server<agv05_navx::ForbiddenLayerConfig> > dsrv_;
};

class StaticPathLayer : public AGV05StaticLayer
{
public:
  StaticPathLayer();
  virtual ~StaticPathLayer();

  virtual void onInitialize(void);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void deactivate();

protected:
  virtual unsigned char gradientCost(double dist);
  virtual void computeCaches();
  virtual void enqueue(double x, double y);
  virtual void enqueue(std::vector<bool>& seen, unsigned int index,
                       unsigned int mx, unsigned int my,
                       unsigned int src_x, unsigned int src_y);
  virtual void dequeue();
  virtual void updateMap(void);

  class Cache
  {
  public:
    explicit Cache(unsigned char cost = costmap_2d::FREE_SPACE) :
      cost_(cost), cells_(std::vector<CellData>())
    {}
    unsigned char cost_;
    std::vector<CellData> cells_;
  };
  std::vector<Cache> cache_;
  bool cache_update_;

  /* ROS subscribers and messages */
  virtual void pathCB(const agv05_msgs::Path::ConstPtr& msg);
  ros::Subscriber sub_;
  agv05_msgs::Path path_;
  struct
  {
    int border_cost;
    int min_i, min_j, max_i, max_j;
    bool bound;
  } area_;

  /* Dynamic reconfigure server */
  virtual void reconfigureCB(agv05_navx::StaticPathLayerConfig &config, uint32_t level);
  boost::shared_ptr< dynamic_reconfigure::Server<agv05_navx::StaticPathLayerConfig> > dsrv_;
  double gradient_dist_;
};



};  // namespace agv05

#endif  // AGV05_NAVX_COSTMAP_LAYERS_H
