#include <amcl/inlier_percentage.h>


float InlierPercentage::computePercentage(const PointCloudT& cloud,
                                          const pf_vector_t& laser_pose,
                                          double inlier_dist)
{
  if (!map_)
  {
    return 0;
  }

  if (inlier_dist < 0.0)
  {
    inlier_dist = inlier_dist_;
  }

  double num_inliers = 0.0;

  for (size_t i = 0, n = cloud.points.size(); i < n; ++i)
  {
    // Convert to map grid coords.
    int mi, mj;
    mi = MAP_GXWX(map_, cloud.points[i].x);
    mj = MAP_GYWY(map_, cloud.points[i].y);

    if (MAP_VALID(map_, mi, mj))
    {
      // Get distance from the hit to closest obstacle.
      double dist = map_->cells[MAP_INDEX(map_, mi, mj)].occ_dist;
      if (dist <= inlier_dist)
      {
        double dx = cloud.points[i].x - laser_pose.v[0];
        double dy = cloud.points[i].y - laser_pose.v[1];
        double obs_range = std::hypot(dx, dy);
        double obs_bearing = std::atan2(dy, dx) - laser_pose.v[2];

        num_inliers += computeInlierScore(obs_range, obs_bearing);
      }
    }
  }

  return num_inliers / cloud.points.size();
}

float InlierPercentage::computePercentage(const amcl::AMCLLaserData& ldata,
                                          const pf_vector_t& laser_pose,
                                          double inlier_dist)
{
  float landmark;
  return computePercentage(computeInlierScore(ldata, laser_pose, inlier_dist), landmark);
}

float InlierPercentage::computePercentage(const InlierPercentage::Score& score, float& landmark)
{
  landmark = 0.0;
  if (score.landmark_hit_)
  {
    for (const auto& inlier : score.landmarks_)
    {
      landmark += inlier;
    }
    return std::min(landmark * z_reflector_ * 
                    score.landmark_hit_ / score.landmark_found_ + score.inlier_, 1.0);
  }
  return score.inlier_;
}

InlierPercentage::Score InlierPercentage::computeInlierScore(const amcl::AMCLLaserData& ldata,
                                                             const pf_vector_t& laser_pose,
                                                             double inlier_dist)
{
  InlierPercentage::Score inlier_score(landmarks_.size());

  if (!map_)
  {
    inlier_score.invalid_ = true;
    return inlier_score;
  }

  if (inlier_dist < 0.0)
  {
    inlier_dist = inlier_dist_;
  }
  double inlier_dist_sq = inlier_dist * inlier_dist;
  double num_inliers = 0.0;
  int& matched_landmark = inlier_score.landmark_hit_;
  std::vector<double>& matched_landmarks = inlier_score.landmarks_;

  for (size_t i = 0, j = 0, n = ldata.range_count; i < n; ++i)
  {
    double obs_range = ldata.ranges[i][0];
    double obs_bearing = ldata.ranges[i][1];

    // This model ignores max range readings
    if (obs_range >= ldata.range_max)
    {
      continue;
    }

    // Check for NaN
    if (obs_range != obs_range)
    {
      continue;
    }

    // Compute the endpoint of the beam
    pf_vector_t hit;
    hit.v[0] = laser_pose.v[0] + obs_range * cos(laser_pose.v[2] + obs_bearing);
    hit.v[1] = laser_pose.v[1] + obs_range * sin(laser_pose.v[2] + obs_bearing);

    // Convert to map grid coords.
    int mi, mj;
    mi = MAP_GXWX(map_, hit.v[0]);
    mj = MAP_GYWY(map_, hit.v[1]);

    if (MAP_VALID(map_, mi, mj))
    {
      // Get distance from the hit to closest obstacle.
      double dist = map_->cells[MAP_INDEX(map_, mi, mj)].occ_dist;
      if (dist <= inlier_dist)
      {
        num_inliers += computeInlierScore(obs_range, obs_bearing);
      }

      if (j < ldata.reflectors.size())
      {
        if (i == ldata.reflectors[j])
        {
          for (size_t k = 0; k < landmarks_.size(); ++k)
          {
            pf_vector_t v = pf_vector_sub(hit, landmarks_[k]);
            double dist_sq = v.v[0] * v.v[0] + v.v[1] * v.v[1];
            if (dist_sq <= inlier_dist_sq)
            {
              double score = computeInlierScore(obs_range, obs_bearing);
              if (matched_landmarks[k] > 0.0)
              {
                if (matched_landmarks[k] < score)
                {
                  matched_landmarks[k] = score;
                }
              }
              else
              {
                matched_landmarks[k] = score;
                ++matched_landmark;
              }
              break;
            }
          }
          ++j;
        }
      }
    }
  }

  inlier_score.inlier_ = num_inliers / ldata.range_count;
  inlier_score.landmark_found_ = ldata.reflectors.size();
  return inlier_score;
}

double InlierPercentage::computeInlierScore(double obs_range, double obs_bearing)
{
  if (decay_dist_ > 0.0)
  {
    if (decay_dist_ < obs_range)
    {
      return decay_dist_ / obs_range;
    }
  }
  return 1.0;
}
