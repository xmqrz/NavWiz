#ifndef AMCL_SNAP_MAP_ICP_H
#define AMCL_SNAP_MAP_ICP_H

#include <amcl/inlier_percentage.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <tf2_ros/buffer.h>

#include <amcl/AMCLConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>


class SnapMapICPImpl;

class SnapMapICP
{
public:
  SnapMapICP();

  void reconfigure(const std::string& base_frame_id, const std::string& global_frame_id, const amcl::AMCLConfig& config);

  void updateMap(const nav_msgs::OccupancyGrid& msg);
  bool processLaser(const sensor_msgs::LaserScan& scan,
                    tf2_ros::Buffer& tf,
                    InlierPercentage& inlier,
                    geometry_msgs::PoseWithCovarianceStamped& pose);

  void icpDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  boost::shared_ptr<SnapMapICPImpl> pimpl_;
};

#endif  // AMCL_SNAP_MAP_ICP_H
