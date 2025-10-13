#ifndef AMCL_REFLECTOR_MATCHER_H
#define AMCL_REFLECTOR_MATCHER_H

#include <amcl/inlier_percentage.h>
#include <amcl/pf/pf_vector.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <tf2_ros/buffer.h>

#include <amcl/AMCLConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class ReflectorMatcher
{
public:
  ReflectorMatcher(std::vector<pf_vector_t>& landmarks) :
    landmarks_(landmarks),
    trigger_(false),
    inlier_perc_(0.0), amcl_inlier_perc_(0.0), corrections_(0)
  {}

  void trigger(bool enable = true)
  {
    trigger_ = enable;
  }

  void reconfigure(const std::string& base_frame_id,
                   const std::string& global_frame_id,
                   const amcl::AMCLConfig& config)
  {
    base_frame_id_ = base_frame_id;
    global_frame_id_ = global_frame_id;

    match_rate_ = config.match_rate;
    match_dist_threshold_ = config.match_dist_threshold;
    match_angle_threshold_ = config.match_angle_threshold;

    search_space_dimension_ = config.search_space_dimension;
    search_space_resolution_ = config.search_space_resolution;
    search_angle_offset_ = config.search_angle_offset;
    search_angle_resolution_ = config.search_angle_resolution;
    match_hit_tolerance_ = config.match_hit_tolerance;

    match_inlier_threshold_ = config.match_inlier_threshold;
    match_inlier_preferable_threshold_ = config.match_inlier_preferable_threshold;
  }

  bool matchReflector(const geometry_msgs::PoseArray& reflectors_msg,
                      const amcl::AMCLLaserData& ldata,
                      tf2_ros::Buffer& tf,
                      InlierPercentage& inlier,
                      geometry_msgs::PoseWithCovarianceStamped& matched_pose);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

private:
  std::vector<pf_vector_t>& landmarks_;

  // dyncfg params
  std::string base_frame_id_, global_frame_id_;
  double match_rate_;
  double match_dist_threshold_, match_angle_threshold_;
  double search_space_dimension_, search_space_resolution_, search_angle_offset_, search_angle_resolution_;
  double match_hit_tolerance_;
  double match_inlier_threshold_, match_inlier_preferable_threshold_;

  bool trigger_;

  std::map<std::string, ros::Time> last_processed_scans_;

  // diagnostics
  double inlier_perc_;
  double amcl_inlier_perc_;
  size_t corrections_;
};

#endif  // AMCL_REFLECTOR_MATCHER_H
