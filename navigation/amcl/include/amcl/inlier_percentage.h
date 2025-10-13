#ifndef AMCL_INLIER_PERCENTAGE_H
#define AMCL_INLIER_PERCENTAGE_H

#include <amcl/map/map.h>
#include <amcl/pf/pf_vector.h>
#include <amcl/sensors/amcl_laser.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <amcl/AMCLConfig.h>


class InlierPercentage
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

public:
  class Score
  {
  public:
    explicit Score(size_t landmarks = 0) :
      invalid_(false), inlier_(0.0),
      landmark_hit_(0), landmark_found_(0), landmarks_(landmarks, 0.0)
    {}

    bool invalid_;
    double inlier_;
    int landmark_hit_;
    int landmark_found_;
    std::vector<double> landmarks_;
  };

public:
  InlierPercentage(const std::vector<pf_vector_t>& landmarks) : map_(0), landmarks_(landmarks)
  {}

  void updateMap(map_t* map)
  {
    map_ = map;
  }

  void reconfigure(amcl::AMCLConfig& config)
  {
    inlier_dist_ = config.inlier_dist;
    decay_dist_ = config.inlier_decay_dist;
    z_reflector_ = config.laser_z_reflector * config.laser_z_reflector * 4;
    z_reflector_ *= (inlier_dist_ / decay_dist_ / M_PI);
  }

  float computePercentage(const PointCloudT& cloud,
                          const pf_vector_t& laser_pose,
                          double inlier_dist = -1.0);

  float computePercentage(const amcl::AMCLLaserData& ldata,
                          const pf_vector_t& laser_pose,
                          double inlier_dist = -1.0);

  float computePercentage(const Score& score, float& landmark);

  Score computeInlierScore(const amcl::AMCLLaserData& ldata,
                           const pf_vector_t& laser_pose,
                           double inlier_dist = -1.0);

private:
  double computeInlierScore(double obs_range, double obs_bearing);

private:
  map_t* map_;
  const std::vector<pf_vector_t>& landmarks_;

  // dyncfg params
  double z_reflector_;
  double inlier_dist_;
  double decay_dist_;
};

#endif  // AMCL_INLIER_PERCENTAGE_H
