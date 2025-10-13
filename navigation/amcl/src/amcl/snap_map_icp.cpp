#include <amcl/snap_map_icp.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>


class SnapMapICPImpl
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

public:
  SnapMapICPImpl();

  void reconfigure(const std::string& base_frame_id, const std::string& global_frame_id, const amcl::AMCLConfig& config);

  void updateMap(const nav_msgs::OccupancyGrid& msg);
  bool processLaser(const sensor_msgs::LaserScan& scan,
                    tf2_ros::Buffer& tf,
                    InlierPercentage& inlier,
                    geometry_msgs::PoseWithCovarianceStamped& pose);

  void icpDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  tf2::Transform matrixAsTransform(const Eigen::Matrix4f &mat)
  {
    double mv[12];
    mv[0] = mat(0, 0);
    mv[4] = mat(0, 1);
    mv[8] = mat(0, 2);
    mv[1] = mat(1, 0);
    mv[5] = mat(1, 1);
    mv[9] = mat(1, 2);
    mv[2] = mat(2, 0);
    mv[6] = mat(2, 1);
    mv[10] = mat(2, 2);

    tf2::Matrix3x3 basis;
    basis.setFromOpenGLSubMatrix(mv);
    tf2::Vector3 origin(mat(0, 3), mat(1, 3), mat(2, 3));
    return tf2::Transform(basis, origin);
  }

  std::string stripSlash(const std::string& in)
  {
    std::string out = in;
    if ((!in.empty()) && (in[0] == '/'))
      out.erase(0, 1);
    return out;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_info_;

  std::string base_frame_id_;
  std::string global_frame_id_;

  double age_threshold_;
  double scan_rate_;
  double update_age_threshold_;
  double dist_threshold_;
  double angle_threshold_;
  double dist_upper_threshold_;
  double angle_upper_threshold_;
  double icp_inlier_threshold_;
  double icp_inlier_preferable_threshold_;
  double icp_inlier_dist_;
  int icp_num_iter_;
  double pose_covariance_trans_;

  boost::shared_ptr<PointCloudT> map_cloud_;

  //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg_;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg_;

  laser_geometry::LaserProjection projector_;

  std::map<std::string, ros::Time> last_processed_scans_;
  ros::Time last_sent_;

  double inlier_perc_;
  double amcl_inlier_perc_;
  size_t corrections_;
};


SnapMapICPImpl::SnapMapICPImpl(): nh_("~"), inlier_perc_(0.0), amcl_inlier_perc_(0.0), corrections_(0)
{
  pub_info_ = nh_.advertise<std_msgs::String>("icp", 1);

  reg_.setTransformationEpsilon(1e-6);
  reg_.setMaxCorrespondenceDistance(0.5);
}

void SnapMapICPImpl::reconfigure(const std::string& base_frame_id, const std::string& global_frame_id, const amcl::AMCLConfig& config)
{
  base_frame_id_ = base_frame_id;
  global_frame_id_ = global_frame_id;

  age_threshold_ = config.age_threshold;
  scan_rate_ = config.scan_rate;
  update_age_threshold_ = config.update_age_threshold;
  dist_threshold_ = config.dist_threshold;
  angle_threshold_ = config.angle_threshold;
  dist_upper_threshold_ = config.dist_upper_threshold;
  angle_upper_threshold_ = config.angle_upper_threshold;
  icp_inlier_threshold_ = config.icp_inlier_threshold;
  icp_inlier_preferable_threshold_ = config.icp_inlier_preferable_threshold;
  icp_inlier_dist_ = config.icp_inlier_dist;
  icp_num_iter_ = config.icp_num_iter;
  pose_covariance_trans_ = config.pose_covariance_trans;

  reg_.setMaximumIterations(icp_num_iter_);
}

void SnapMapICPImpl::updateMap(const nav_msgs::OccupancyGrid& msg)
{
  float resolution = msg.info.resolution;
  float width = msg.info.width;
  float height = msg.info.height;

  float posx = msg.info.origin.position.x;
  float posy = msg.info.origin.position.y;

  map_cloud_ = boost::make_shared<PointCloudT>();

  map_cloud_->height = 1;
  map_cloud_->is_dense = false;
  std_msgs::Header header;
  header.stamp = ros::Time(0);
  header.frame_id = global_frame_id_;
  map_cloud_->header = pcl_conversions::toPCL(header);

  pcl::PointXYZ p;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      if (msg.data[x + y * width] == 100)
      {
        p.x = (.5f + x) * resolution + posx;
        p.y = (.5f + y) * resolution + posy;
        p.z = 0;
        map_cloud_->points.push_back(p);
      }
    }
  }
  map_cloud_->width = map_cloud_->points.size();
}

bool SnapMapICPImpl::processLaser(const sensor_msgs::LaserScan& scan,
                                  tf2_ros::Buffer& tf,
                                  InlierPercentage& inlier,
                                  geometry_msgs::PoseWithCovarianceStamped& pose)
{
  ros::Time scan_time = scan.header.stamp;
  ros::Time now = ros::Time::now();

  ros::Time last_processed_scan;
  if (last_processed_scans_.find(scan.header.frame_id) != last_processed_scans_.end())
  {
    last_processed_scan = last_processed_scans_[scan.header.frame_id];
  }
  if (!scan_rate_ || scan_time - last_processed_scan < ros::Duration(1.0f / scan_rate_))
  {
    return false;
  }
  last_processed_scans_[scan.header.frame_id] = scan_time;

  ros::Duration scan_age = now - scan_time;
  if (scan_age.toSec() > age_threshold_)
  {
    ROS_WARN("SCAN SEEMS TOO OLD (%f seconds, %f threshold) scan time: %f, now %f", scan_age.toSec(), age_threshold_, scan_time.toSec(), now.toSec());
    return false;
  }

  // project laser scan to PointCloud format in the map frame
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2 cloud_in_map;
  PointCloudT::Ptr pcl_cloud(new PointCloudT());

  projector_.projectLaser(scan, cloud);

  geometry_msgs::TransformStamped map_to_cloud;
  try
  {
    map_to_cloud = tf.lookupTransform(global_frame_id_, stripSlash(cloud.header.frame_id), cloud.header.stamp, ros::Duration(0.05));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("No map to cloud transform found");
    return false;
  }
  tf2::doTransform(cloud, cloud_in_map, map_to_cloud);

  pcl::fromROSMsg(cloud_in_map, *pcl_cloud);
  for (size_t k = 0; k < pcl_cloud->points.size(); k++)
  {
    pcl_cloud->points[k].z = 0;
  }

  // lookup robot pose
  tf2::Stamped<tf2::Transform> old_pose;
  try
  {
    tf2::fromMsg(tf.lookupTransform(global_frame_id_, base_frame_id_, cloud.header.stamp, ros::Duration(0.05)), old_pose);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Fail to get original pose in map");
    return false;
  }

  // run ICP point cloud matching
  reg_.setInputSource(pcl_cloud);
  reg_.setInputTarget(map_cloud_);

  PointCloudT transformed_cloud;
  reg_.align(transformed_cloud);
  ROS_ASSERT(transformed_cloud.points.size() == pcl_cloud->points.size());

  tf2::Transform t = matrixAsTransform(reg_.getFinalTransformation());

  // compute inlier percentages
  pf_vector_t laser_pose;
  laser_pose.v[0] = map_to_cloud.transform.translation.x;
  laser_pose.v[1] = map_to_cloud.transform.translation.y;
  laser_pose.v[2] = tf2::getYaw(map_to_cloud.transform.rotation);
  amcl_inlier_perc_ = inlier.computePercentage(*pcl_cloud, laser_pose, icp_inlier_dist_);
  inlier_perc_ = inlier.computePercentage(transformed_cloud, laser_pose, icp_inlier_dist_);

  // compute new pose
  t = t * old_pose;

  // compute change between old and new pose
  double dx = t.getOrigin().x() - old_pose.getOrigin().x();
  double dy = t.getOrigin().y() - old_pose.getOrigin().y();
  double dist = std::hypot(dx, dy);
  double angle_dist = tf2::getYaw(t.getRotation()) - tf2::getYaw(old_pose.getRotation());
  angle_dist = std::atan2(std::sin(angle_dist), std::cos(angle_dist));  // normalize angle
  angle_dist = std::abs(angle_dist);  // remove sign

  char msg_c_str[2048];
  sprintf(msg_c_str, "INLIERS %f/%f (%f) scan_age %f dist %f angleDist %f converged %i processing %f",
          inlier_perc_, amcl_inlier_perc_, icp_inlier_threshold_, scan_age.toSec(), dist, angle_dist,
          reg_.hasConverged(), (ros::Time::now() - now).toSec());
  std_msgs::String strmsg;
  strmsg.data = msg_c_str;

  if ((now - last_sent_).toSec() > update_age_threshold_ &&
      (dist > dist_threshold_ || angle_dist > angle_threshold_) &&
      dist < dist_upper_threshold_ && angle_dist < angle_upper_threshold_ &&
      inlier_perc_ > icp_inlier_threshold_ &&
      inlier_perc_ - amcl_inlier_perc_ > icp_inlier_preferable_threshold_ &&
      reg_.hasConverged())
  {
    ++corrections_;
    last_sent_ = now;
    double cov = pose_covariance_trans_;

    pose.header.stamp = cloud.header.stamp;
    pose.header.frame_id = global_frame_id_;
    pose.pose.pose.position.x = t.getOrigin().x();
    pose.pose.pose.position.y = t.getOrigin().y();

    tf2::Quaternion quat = t.getRotation();
    pose.pose.pose.orientation = tf2::toMsg(quat);
    float factorPos = 0.03;
    float factorRot = 0.1;
    pose.pose.covariance[6 * 0 + 0] = cov * cov * factorPos;
    pose.pose.covariance[6 * 1 + 1] = cov * cov * factorPos;
    pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0 * factorRot;
    strmsg.data += " << SENT";
    pub_info_.publish(strmsg);
    return true;
  }
  else
  {
    pub_info_.publish(strmsg);
    return false;
  }
}

void SnapMapICPImpl::icpDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.addf("ICP Inlier (%)", "%.2f", inlier_perc_ * 100);
  stat.addf("AMCL Inlier (%)", "%.2f", amcl_inlier_perc_ * 100);
  stat.add("ICP Corrections", corrections_);
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}


/* SnapMapICP class */
SnapMapICP::SnapMapICP():
  pimpl_(new SnapMapICPImpl())
{
}

void SnapMapICP::reconfigure(const std::string& base_frame_id, const std::string& global_frame_id, const amcl::AMCLConfig& config)
{
  pimpl_->reconfigure(base_frame_id, global_frame_id, config);
}

void SnapMapICP::updateMap(const nav_msgs::OccupancyGrid& msg)
{
  pimpl_->updateMap(msg);
}

bool SnapMapICP::processLaser(const sensor_msgs::LaserScan& scan,
                              tf2_ros::Buffer& tf,
                              InlierPercentage& inlier,
                              geometry_msgs::PoseWithCovarianceStamped& pose)
{
  return pimpl_->processLaser(scan, tf, inlier, pose);
}

void SnapMapICP::icpDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  pimpl_->icpDiagnostics(stat);
}
