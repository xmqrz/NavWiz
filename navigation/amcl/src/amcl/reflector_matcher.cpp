#include <amcl/reflector_matcher.h>

#include <tf2/utils.h>


bool ReflectorMatcher::matchReflector(const geometry_msgs::PoseArray& reflectors_msg,
                                      const amcl::AMCLLaserData& ldata,
                                      tf2_ros::Buffer& tf,
                                      InlierPercentage& inlier,
                                      geometry_msgs::PoseWithCovarianceStamped& matched_pose)
{
  ros::Time scan_time = reflectors_msg.header.stamp;
  ros::Time now = ros::Time::now();

  if (!trigger_)
  {
    ros::Time last_processed_scan;
    if (last_processed_scans_.find(reflectors_msg.header.frame_id) != last_processed_scans_.end())
    {
      last_processed_scan = last_processed_scans_[reflectors_msg.header.frame_id];
    }
    if (!match_rate_ || scan_time - last_processed_scan < ros::Duration(1.0f / match_rate_))
    {
      return false;
    }
  }
  last_processed_scans_[reflectors_msg.header.frame_id] = scan_time;

  // need a minimum of two reflectors to eliminate ambiguity
  if (reflectors_msg.poses.size() < 2)
  {
    return false;
  }

  // lookup robot pose
  geometry_msgs::TransformStamped map_to_base;
  try
  {
    map_to_base = tf.lookupTransform(global_frame_id_, base_frame_id_, scan_time, ros::Duration(0.05));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("No map to base transform found");
    return false;
  }
  pf_vector_t robot_pose;
  robot_pose.v[0] = map_to_base.transform.translation.x;
  robot_pose.v[1] = map_to_base.transform.translation.y;
  robot_pose.v[2] = tf2::getYaw(map_to_base.transform.rotation);

  // lookup laser in base
  geometry_msgs::TransformStamped base_to_laser;
  try
  {
    base_to_laser = tf.lookupTransform(base_frame_id_, reflectors_msg.header.frame_id, scan_time, ros::Duration(0.05));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("No base to laser transform found");
    return false;
  }

  // project reflectors to robot's base frame
  std::vector<pf_vector_t> reflectors_in_base;
  for (int i = 0, n = reflectors_msg.poses.size(); i < n; ++i)
  {
    geometry_msgs::Point p;
    tf2::doTransform(reflectors_msg.poses[i].position, p, base_to_laser);

    pf_vector_t v;
    v.v[0] = p.x;
    v.v[1] = p.y;
    v.v[2] = 0;
    reflectors_in_base.push_back(v);
  }

  // calculate angle arrays
  std::vector<float> th_poses;
  th_poses.push_back(robot_pose.v[2]);
  for (int i = 1, n = search_angle_offset_ / 2 / search_angle_resolution_; i <= n; ++i)
  {
    float offset = i * search_angle_resolution_;
    th_poses.push_back(robot_pose.v[2] + offset);
    th_poses.push_back(robot_pose.v[2] - offset);
  }

  // calculate position arrays
  std::vector<float> x_poses;
  std::vector<float> y_poses;
  x_poses.push_back(robot_pose.v[0]);
  y_poses.push_back(robot_pose.v[1]);
  for (int i = 1, n = search_space_dimension_ / 2 / search_space_resolution_; i <= n; ++i)
  {
    float offset = i * search_space_resolution_;
    x_poses.push_back(robot_pose.v[0] + offset);
    x_poses.push_back(robot_pose.v[0] - offset);
    y_poses.push_back(robot_pose.v[0] + offset);
    y_poses.push_back(robot_pose.v[0] - offset);
  }

  // iterate search space
  float hit_sq = match_hit_tolerance_ * match_hit_tolerance_;
  int max_hits = 0;
  float min_error = 1000000;
  pf_vector_t best_pose;

  typedef std::vector<float>::iterator iter;
  typedef std::vector<pf_vector_t>::iterator pf_vector_iter;

  for (iter tt = th_poses.begin(), tt_end = th_poses.end(); tt != tt_end; ++tt)
  {
    float th = *tt;
    float sin_th = std::sin(th);
    float cos_th = std::cos(th);
    std::vector<pf_vector_t> rotated_reflectors;

    // rotate reflectors about the search angle (th) around the base frame
    for (pf_vector_iter it = reflectors_in_base.begin(), it_end = reflectors_in_base.end(); it != it_end; ++it)
    {
      pf_vector_t v;
      v.v[0] = it->v[0] * cos_th - it->v[1] * sin_th;
      v.v[1] = it->v[0] * sin_th + it->v[1] * cos_th;
      v.v[2] = 0;
      rotated_reflectors.push_back(v);
    }

    // search in x and y space
    for (iter xt = x_poses.begin(), xt_end = x_poses.end(); xt != xt_end; ++xt)
    {
      float x = *xt;
      for (iter yt = y_poses.begin(), yt_end = y_poses.end(); yt != yt_end; ++yt)
      {
        float y = *yt;

        int hits = 0;
        float error = 0;
        for (pf_vector_iter rt = rotated_reflectors.begin(), rt_end = rotated_reflectors.end(); rt != rt_end; ++rt)
        {
          float rx = rt->v[0] + x;
          float ry = rt->v[1] + y;

          float min_d2 = 1000000;
          for (pf_vector_iter lt = landmarks_.begin(), lt_end = landmarks_.end(); lt != lt_end; ++lt)
          {
            float dx = lt->v[0] - rx;
            float dy = lt->v[1] - ry;
            float d2 = dx * dx + dy * dy;
            min_d2 = std::min(d2, min_d2);
          }

          if (min_d2 <= hit_sq)
          {
            ++hits;
            error += min_d2;
          }
        }

        if (hits > max_hits || hits == max_hits && error < min_error)
        {
          max_hits = hits;
          min_error = error;
          best_pose.v[0] = x;
          best_pose.v[1] = y;
          best_pose.v[2] = th;
        }
      }
    }
  }

  // need a minimum of two reflectors matching to eliminate ambiguity
  if (max_hits < 2)
  {
    return false;
  }

  // compute laser pose in map
  pf_vector_t laser_pose;
  laser_pose.v[0] = base_to_laser.transform.translation.x;
  laser_pose.v[1] = base_to_laser.transform.translation.y;
  laser_pose.v[2] = 0;  // laser mounting angle incorporated in ldata -> set to 0 here!

  pf_vector_t laser_pose_old = pf_vector_coord_add(laser_pose, robot_pose);
  pf_vector_t laser_pose_corrected = pf_vector_coord_add(laser_pose, best_pose);

  // compute inlier percentages and check thresholds
  amcl_inlier_perc_ = inlier.computePercentage(ldata, laser_pose_old);
  inlier_perc_ = inlier.computePercentage(ldata, laser_pose_corrected);

  if (inlier_perc_ < match_inlier_threshold_ ||
      match_inlier_preferable_threshold_ &&
      inlier_perc_ - amcl_inlier_perc_ < match_inlier_preferable_threshold_)
  {
    return false;
  }

  // check conditions for publishing pose correction
  if (!trigger_)
  {
    pf_vector_t diff_pose = pf_vector_sub(best_pose, robot_pose);
    float dist = hypot(diff_pose.v[0], diff_pose.v[1]);
    float angle_dist = diff_pose.v[2];

    if (dist < match_dist_threshold_ && angle_dist < match_angle_threshold_)
    {
      return false;
    }
  }

  // convert best_pose to PoseWithCovarianceStamped
  matched_pose.header.stamp = scan_time;
  matched_pose.header.frame_id = global_frame_id_;
  matched_pose.pose.pose.position.x = best_pose.v[0];
  matched_pose.pose.pose.position.y = best_pose.v[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, best_pose.v[2]);
  tf2::convert(q, matched_pose.pose.pose.orientation);

  matched_pose.pose.covariance[0] = 0.01;
  matched_pose.pose.covariance[7] = 0.01;
  matched_pose.pose.covariance[35] = M_PI / 36.0 * M_PI / 36.0;

  trigger_ = false;
  ++corrections_;
  return true;
}

void ReflectorMatcher::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.addf("Reflector Inlier (%)", "%.2f", inlier_perc_ * 100);
  stat.addf("AMCL Inlier (%)", "%.2f", amcl_inlier_perc_ * 100);
  stat.add("Reflector Corrections", corrections_);
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
}
