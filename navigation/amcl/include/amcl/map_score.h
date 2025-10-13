#ifndef AMCL_MAP_SCORE_H
#define AMCL_MAP_SCORE_H

#include <amcl/inlier_percentage.h>
#include <amcl/pf/pf_vector.h>
#include <amcl/sensors/amcl_laser.h>

#include <boost/thread.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

#include <amcl/AMCLConfig.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>


struct QualityMap;

class MapScore
{
public:
  explicit MapScore(const std::string& map_layout) :
    enable_(false),
    inlier_overall_(0.0f), inlier_landmark_(0.0f),
    map_layout_(map_layout),
    thread_shutdown_(false)
  {
    loadQmaps();
    thread_ = boost::thread(&MapScore::writerThread, this);
  }

  ~MapScore()
  {
    {
      Lock lock(mutex_);
      thread_shutdown_ = true;
      cv_.notify_one();
    }
    thread_.join();
  }

  void reconfigure(amcl::AMCLConfig& config);

  void updateEnable(bool enable)
  {
    if (enable_ != enable)
    {
      enable_ = enable;
      cv_.notify_one();
    }
  }

  void updateMap(const nav_msgs::OccupancyGrid& msg, const std::string& map_layout);
  void updateMap(const nav_msgs::OccupancyGrid& msg);

  void update(const geometry_msgs::Pose& robot_pose,
              const std::string& laser_scan_frame_id,
              const pf_vector_t& laser_pose,
              const amcl::AMCLLaserData& ldata,
              InlierPercentage& inlier);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  void loadQmaps();
  void writerThread();

private:
  bool enable_;

  // dyncfg params
  bool new_qmap_;

  // diagnostics
  std::map< std::string, InlierPercentage::Score > inlier_perc_;
  float inlier_overall_;
  float inlier_landmark_;

  // map
  const std::string& map_layout_;

  // quality map
  std::vector<boost::shared_ptr<QualityMap> > qmaps_;
  boost::shared_ptr<QualityMap> current_qmap_;
  std::string today_;

  // writer thread
  boost::thread thread_;
  bool thread_shutdown_;

  // mutex
  typedef boost::mutex::scoped_lock Lock;
  boost::mutex mutex_;       // protects qmaps_ and cv_
  boost::mutex mutex_fast_;  // protects current_qmap_
  boost::condition_variable cv_;
};

#endif  // AMCL_MAP_SCORE_H
