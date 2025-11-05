/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include <marker_localization/detectors/pallet.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_set>
#include <visualization_msgs/Marker.h>


namespace marker_localization
{
using boost::make_shared;

const float NaN = std::numeric_limits<float>::quiet_NaN();

void PalletDetector::start()
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  Detector::start(nh, true, false);
  if (!cloud_voxel_pub_)
  {
    nh.setParam("depth_voxel/compressed/format", "png");
    nh.setParam("depth_voxel/compressed/png_level", 1);
    nh.setParam("depth_voxel/compressedDepth/depth_max", 10);
    nh.setParam("depth_voxel/compressedDepth/png_level", 1);
    nh.setParam("depth_vertical/compressed/format", "png");
    nh.setParam("depth_vertical/compressed/png_level", 1);
    nh.setParam("depth_vertical/compressedDepth/depth_max", 10);
    nh.setParam("depth_vertical/compressedDepth/png_level", 1);

    // publishers are static variables and only registered once
    // publisher shutdown is avoided as it takes time for its peers to resubscribe
    cloud_voxel_pub_ = nh.advertise<PointCloud>("cloud_voxel", 1);
    cloud_vertical_pub_ = nh.advertise<PointCloud>("cloud_vertical", 1);
    cloud_colored_pub_ = nh.advertise<PointCloud>("cloud_colored", 1);
    cloud_deck_pub_ = nh.advertise<PointCloud>("cloud_deck", 1);
    marker_viz_pub_ = nh.advertise<visualization_msgs::Marker>("marker_viz", 1);
    depth_voxel_pub_ = it.advertise("depth_voxel", 1);
    depth_vertical_pub_ = it.advertise("depth_vertical", 1);
  }

  // determine trim angles and range
  // possible string formats:
  //  {sensor}__{totalAngle} or {sensor}__{angleMin}_{angleMax} or {sensor}__r{rangeMax} or
  //  {sensor}__{totalAngle}__r{rangeMax} or {sensor}__{angleMin}_{angleMax}__r{rangeMax}
  // input units: angle -> degrees, range -> meters
  trim_angle_min_ = -10;  // in radians
  trim_angle_max_ = 10;  // in radians
  trim_range_max_ = std::numeric_limits<uint16_t>::max();  // in millimeters

  std::string angle_str;
  int index = sensor_.find("__");
  if (index != std::string::npos)
  {
    angle_str = sensor_.substr(index + 2);
    sensor_ = sensor_.substr(0, index);
  }
  std::string range_str;
  index = angle_str.find("__r");
  if (index != std::string::npos)
  {
    range_str = angle_str.substr(index + 3);
    angle_str = angle_str.substr(0, index);
  }
  else if (angle_str.front() == 'r')
  {
    range_str = angle_str.substr(1);
    angle_str.clear();
  }

  if (!angle_str.empty())
  {
    index = angle_str.find("_");
    if (index == std::string::npos)
    {
      std::istringstream iss(angle_str);
      float angle;

      if (iss >> angle)
      {
        angle = fabs(angle);
        angle *= M_PI / 180 / 2;
        trim_angle_min_ = -angle;
        trim_angle_max_ = angle;
      }
    }
    else
    {
      std::istringstream iss(angle_str.substr(0, index));
      std::istringstream iss2(angle_str.substr(index + 1));
      float angle_min;
      float angle_max;

      if ((iss >> angle_min) && (iss2 >> angle_max))
      {
        trim_angle_min_ = angle_min * M_PI / 180;
        trim_angle_max_ = angle_max * M_PI / 180;
      }
    }
  }

  if (!range_str.empty())
  {
    std::istringstream iss(range_str);
    float range;

    if (iss >> range)
    {
      trim_range_max_ = fabs(range) * 1000;
    }
  }

  std::string topic = getCameraTopic(sensor_);
  if (topic.length())
  {
    // Update camera model
    auto info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
                  topic + "/depth/camera_info", nh, ros::Duration(2.0));
    if (!info)
    {
      ROS_ERROR_STREAM("No camera info: " << sensor_);
      return;
    }
    model_.fromCameraInfo(info);

    depth_sub_ = it.subscribe(topic + "/depth/image_rect_raw", 1, &PalletDetector::callbackDepth, this,
                              image_transport::TransportHints("compressed"));
  }
  else
  {
    ROS_ERROR_STREAM("No sensor found: " << sensor_);
  }
}

void PalletDetector::stop()
{
  depth_sub_.shutdown();

  auto cloud = make_shared<PointCloud>();
  cloud_voxel_pub_.publish(cloud);
  cloud_vertical_pub_.publish(cloud);
  cloud_colored_pub_.publish(cloud);

  Detector::stop();
}

void PalletDetector::callbackDepth(const sensor_msgs::ImageConstPtr& depth)
{
  ROS_DEBUG("Depth received");

#if 0
  static bool once = false;
  if (once) return;
  once = true;
#endif

  // Transform to base frame
  geometry_msgs::TransformStamped camera_transform;
  try
  {
    camera_transform = tf_.lookupTransform("base", depth->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Camera transform error: " << ex.what());
    return;
  }
  tf2::Transform camera_tf;
  tf2::fromMsg(camera_transform.transform, camera_tf);
  const float camera_x = camera_transform.transform.translation.x;

  voxelFilter(depth, camera_tf) && verticalFilter(camera_x) && cluster(camera_x);
}

bool PalletDetector::voxelFilter(const sensor_msgs::ImageConstPtr& depth, const tf2::Transform& camera_tf)
{
  // projection model
  const float center_x = model_.cx();
  const float center_y = model_.cy();
  const double unit_scaling = 0.001;  // originally 1 unit = 1mm
  const float constant_x = unit_scaling / model_.fx();
  const float constant_y = unit_scaling / model_.fy();
  ROS_DEBUG("Projection Model: %f %f", 1.0 / model_.fx(), 1.0 / model_.fy());

  // Project and crop
  ROS_ASSERT(depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1);
  const int width = depth->width;
  const int height = depth->height;
  const int depth_row_step = depth->step / sizeof(uint16_t);
  const int cloud_row_step = width;

  int u_start = (trim_angle_min_ > -M_PI / 2) ? tan(trim_angle_min_) * model_.fx() + center_x : 0;
  int u_end = (trim_angle_max_ < M_PI / 2) ? tan(trim_angle_max_) * model_.fx() + center_x + 1 : width;
  u_start = std::max(u_start, 0);
  u_end = std::min(u_end, width);

  depth_voxel_ = *depth;
  uint16_t* depth_row = reinterpret_cast<uint16_t*>(&depth_voxel_.data[0]);

  cloud_voxel_ = make_shared<PointCloud0>(width, height);
  cloud_voxel_->header.frame_id = "base";

  voxel_count_ = 0;
  for (int v = 0; v < height; ++v)
  {
    for (int u = u_start; u < u_end; ++u)
    {
      uint16_t& d = depth_row[u];
      pcl::PointXYZ& p = cloud_voxel_->points[v * cloud_row_step + u];

      if (d == 0 || d > trim_range_max_)
      {
        p.x = NaN;
        p.y = NaN;
        p.z = NaN;
        d = 0;
        continue;
      }

      float x = (u - center_x) * d * constant_x;
      float y = (v - center_y) * d * constant_y;
      float z = d * unit_scaling;
      tf2::Vector3 t = camera_tf * tf2::Vector3(x, y, z);

      // filter by min and max Z
      if (t.z() < *config_pallet_.crop_z_min || t.z() > *config_pallet_.crop_z_max)
      {
        p.x = NaN;
        p.y = NaN;
        p.z = NaN;
        d = 0;
        continue;
      }

      p.x = t.x();
      p.y = t.y();
      p.z = t.z();
      ++voxel_count_;
    }
    depth_row += depth_row_step;
  }

  depth_voxel_pub_.publish(depth_voxel_);
  if (cloud_voxel_pub_.getNumSubscribers() > 0)
  {
    cloud_voxel_pub_.publish(cloud_voxel_);
  }
  if (voxel_count_ == 0)
  {
    ROS_DEBUG("Empty cloud voxel");
    return false;
  }
  return true;
}

bool PalletDetector::verticalFilter(const float camera_x)
{
  // Detect vertical surfaces
  const float sine_vertical_min = sin(*config_pallet_.block_vertical_angle_min / 180.0 * M_PI);
  const float sine_vertical_max = sin(*config_pallet_.block_vertical_angle_max / 180.0 * M_PI);

  ROS_ASSERT(depth_voxel_.encoding == sensor_msgs::image_encodings::TYPE_16UC1);
  const int width = depth_voxel_.width;
  const int height = depth_voxel_.height;
  const int depth_row_step = depth_voxel_.step / sizeof(uint16_t);
  const int cloud_row_step = width;

  depth_vertical_.header = depth_voxel_.header;
  depth_vertical_.height = depth_voxel_.height;
  depth_vertical_.width = depth_voxel_.width;
  depth_vertical_.encoding = depth_voxel_.encoding;
  depth_vertical_.is_bigendian = depth_voxel_.is_bigendian;
  depth_vertical_.step = depth_voxel_.step;
  depth_vertical_.data.resize(width * height * sizeof(uint16_t), 0);

  uint16_t* depth_voxel_data = reinterpret_cast<uint16_t*>(&depth_voxel_.data[0]);
  uint16_t* depth_vertical_data = reinterpret_cast<uint16_t*>(&depth_vertical_.data[0]);

  cloud_vertical_ = boost::make_shared<PointCloud0>(width, height, pcl::PointXYZ(NaN, NaN, NaN));
  cloud_vertical_->header.frame_id = "base";

  int vertical_count_ = 0;
  for (int u = 0; u < width; ++u)
  {
    pcl::PointXYZ* prev_p = &cloud_voxel_->points[u];
    pcl::PointXYZ* prev_q = &cloud_vertical_->points[u];
    uint16_t* prev_d = &depth_voxel_data[u];
    uint16_t* prev_e = &depth_vertical_data[u];

    for (int v = 1; v < height; ++v)
    {
      pcl::PointXYZ& p = cloud_voxel_->points[v * cloud_row_step + u];
      pcl::PointXYZ& q = cloud_vertical_->points[v * cloud_row_step + u];
      uint16_t& d = depth_voxel_data[v * depth_row_step + u];
      uint16_t& e = depth_vertical_data[v * depth_row_step + u];

      if (!isnan(prev_p->x))
      {
        float dx = p.x - prev_p->x;
        float dz = p.z - prev_p->z;
        if (dz < 0)  // height must decrease
        {
          // compute normals
          float nx = -dz;
          float nz = dx;
          if (nx * (p.x - camera_x) > 0)
          {
            nx = dz;
            nz = -dx;
          }
          float n = sqrt(nx * nx + nz * nz);
          nx /= n;
          nz /= n;

          if (nz >= sine_vertical_min && nz <= sine_vertical_max)
          {
            if (isnan(prev_q->x))
            {
              *prev_q = *prev_p;
              *prev_e = *prev_d;
              ++vertical_count_;
            }
            q = p;
            e = d;
            ++vertical_count_;
          }
        }
      }
      else
      {
        e = 0;
      }
      prev_p = &p;
      prev_q = &q;
      prev_d = &d;
      prev_e = &e;
    }
  }

  depth_vertical_pub_.publish(depth_vertical_);
  if (cloud_vertical_pub_.getNumSubscribers() > 0)
  {
    cloud_vertical_pub_.publish(cloud_vertical_);
  }
  if (!vertical_count_)
  {
    ROS_DEBUG("Sizes: %d -> %d", width * height, voxel_count_);
    ROS_DEBUG("Empty vertical cloud");
    return false;
  }
  return true;
}

bool PalletDetector::cluster(const float camera_x)
{
  // Region Growing Segmentation
  struct VStrip
  {
    int u;
    int center_v;
    Eigen::Vector2f centroid;
    VStrip* left_pair;
    VStrip* right_pair;
    VStrip* group_head;

    VStrip() :
      u(0), center_v(0), centroid(Eigen::Vector2f::Zero()), left_pair(NULL), right_pair(NULL), group_head(NULL)
    {}
    void mark(int u0, int start, int end)
    {
      u = u0;
      center_v = (start + end) >> 1;
    }
  };

  struct VStripU
  {
    int n, s, e;
    Eigen::Vector2f centroid;
    struct {float start, end;} z;

    explicit VStripU(int start, int end, float z_start, float z_end) :
      n(0), s(start), e(end), centroid(Eigen::Vector2f::Zero()), z({z_start, z_end})
    {}
    void merge(VStripU* p, float height_max, float gap_max)
    {
      if (!(n && p->n)) return;

      float start = std::max(z.start, p->z.start);
      float end = std::min(z.end, p->z.end);
      if ((start - end) > height_max) return;

      float gap_x = centroid.x() / n - p->centroid.x() / p->n;
      if (fabs(gap_x) > gap_max) return;

      s = std::min(s, p->s);
      e = std::max(e, p->e);
      z.start = start;
      z.end = end;
      centroid += p->centroid;
      n += p->n;
      p->n = 0;
    }
  };

  const int width = depth_vertical_.width;
  const int height = depth_vertical_.height;
  const int cloud_row_step = width;

  auto cloud_flatten = make_shared<PointCloud0>();
  cloud_flatten->header.frame_id = "base";
  cloud_flatten->points.reserve(width);

  std::vector<std::vector<VStrip>> all_strips(width);
  int strip_count = 0;
  for (int u = 0; u < width; ++u)
  {
    int start = -1, end;
    std::vector<VStripU> strip_u;

    for (int v = 0; v <= height; ++v)
    {
      if (v < height)
      {
        pcl::PointXYZ& p = cloud_vertical_->points[v * cloud_row_step + u];

        if (isnan(p.x))
        {
          continue;
        }

        if (start < 0)
        {
          start = v;
          end = v;
          continue;
        }
      }
      else if (start < 0)
      {
        continue;
      }

      int index_end = end * cloud_row_step + u;
      pcl::PointXYZ& p_end = cloud_vertical_->points[index_end];

      bool update = true;
      if (v < height)
      {
        pcl::PointXYZ& p = cloud_vertical_->points[v * cloud_row_step + u];
        float gap_x = p_end.x - p.x;
        float gap_z = p_end.z - p.z;
        update = (gap_z < 0 || fabs(gap_x) >= 0.04f);  // check gap to separate strips
      }
      if (update)
      {
        int index = start * cloud_row_step + u;
        pcl::PointXYZ& p_start = cloud_vertical_->points[index];
        VStripU s(start, end, p_start.z, p_end.z);

        for (; index <= index_end; index += cloud_row_step)
        {
          pcl::PointXYZ& p = cloud_vertical_->points[index];
          if (!isnan(p.x))
          {
            s.centroid += p.getVector3fMap().head<2>();
            s.n++;
          }
        }
        strip_u.push_back(s);
        start = -1;
      }
      else
      {
        end = v;
      }
    }

    for (int i = 0; i < strip_u.size(); ++i)
    {
      if (!strip_u[i].n) continue;
      for (int j = 0; j < strip_u.size(); ++j)
      {
        if (i == j) continue;
        strip_u[i].merge(&strip_u[j], *config_pallet_.block_height_max, *config_pallet_.block_width_max);
      }
    }

    std::vector<VStrip>& strips = all_strips[u];
    for (auto& strip : strip_u)
    {
      if (strip.n)
      {
        float h = strip.z.start - strip.z.end;  // height of the vertical strip
        if (h >= *config_pallet_.block_height_min && h <= *config_pallet_.block_height_max)
        {
          if (strip.n >= (strip.e - strip.s + 1) * 0.5)  // 50 percent filled
          {
            VStrip s;
            s.centroid = strip.centroid / strip.n;
            s.mark(u, strip.s, strip.e);
            cloud_flatten->points.emplace_back(s.centroid.x(), s.centroid.y(), 0.0f);
            strips.push_back(s);
            ++strip_count;
          }
        }
      }
    }
  }

  if (cloud_flatten_pub_.getNumSubscribers() > 0)
  {
    cloud_flatten->width = cloud_flatten->points.size();
    cloud_flatten->height = 1;
    cloud_flatten_pub_.publish(cloud_flatten);
  }

  // Clustering
  std::unordered_set<VStrip*> heads;
  // const float cosine_smoothness = cos(*config_pallet_.block_smoothness_angle / 180.0 * M_PI);
  const float max_sq_gap = *config_pallet_.block_width_min * *config_pallet_.block_width_min;
  const int MAX_GAP_HORIZONTAL = 4;

  for (int u = MAX_GAP_HORIZONTAL; u < width; ++u)
  {
    auto& strips = all_strips[u];

    for (int v = 0; v < MAX_GAP_HORIZONTAL; ++v)
    {
      auto& prev_strips = all_strips[u - v - 1];
      int i = 0, j = 0;
      for (; i < strips.size(); ++i)
      {
        VStrip& s = strips[i];
        if (s.left_pair) continue;

        int k = 0;
        while (j + k < prev_strips.size())
        {
          VStrip& prev_s = prev_strips[j + k];
          int dv = prev_s.center_v - s.center_v;

          if (dv > 6)
          {
            break;
          }
          if (dv < -6)
          {
            ++j;
            continue;
          }
          if (prev_s.right_pair)
          {
            ++k;
            continue;
          }

          // check if valid neighbours
          Eigen::Vector2f d = prev_s.centroid - s.centroid;
          if (d.dot(d) < max_sq_gap)  // distance
          {
            s.left_pair = &prev_s;
            s.group_head = &prev_s;
            prev_s.right_pair = &s;

            if (prev_s.left_pair)
            {
              // compare normals
              // Eigen::Vector2f prev_d = prev_s.left_pair->centroid - prev_s.centroid;
              // float dot = d.dot(prev_d) / sqrt(d.dot(d) * prev_d.dot(prev_d));

              // if (fabs(dot) > cosine_smoothness)
              // {
              s.group_head = prev_s.group_head;
              // }
            }
            heads.insert(s.group_head);
            j += k + 1;
            break;
          }
          ++k;
        }
      }
    }
  }

  int region_count = 0;
  uint32_t rgba[20] =
  {
    0x1f77b4, 0xaec7e8,
    0xff7f0e, 0xffbb78,
    0x2ca02c, 0x98df8a,
    0xd62728, 0xff9896,
    0x9467bd, 0xc5b0d5,
    0x8c564b, 0xc49c94,
    0xe377c2, 0xf7b6d2,
    0x7f7f7f, 0xc7c7c7,
    0xbcbd22, 0xdbdb8d,
    0x17becf, 0x9edae5
  };

  std::vector<std::vector<Eigen::Vector2f>> centcentroids;
  std::vector<Eigen::Vector2f> centroids;
  std::vector<Eigen::Vector2f> medians;
  std::vector<Eigen::Vector2f> minimas;
  std::vector<std::pair<int, int>> columns;

  auto cloud_segmented = make_shared<PointCloud>();
  cloud_segmented->header.frame_id = "base";
  cloud_segmented->points.reserve(strip_count);

  for (auto& head : heads)
  {
    Eigen::Vector2f minima = head->centroid;
    const VStrip *s = head, *tail = head;
    while (s->right_pair)
    {
      s = s->right_pair;
      if (fabs(s->centroid.x() - camera_x) < fabs(minima.x() - camera_x))
      {
        minima = s->centroid;
        tail = s;
      }
    }

    auto cluster = [&](const VStrip* h, const VStrip* t, Eigen::Vector2f v) -> bool  // NOLINT(build/c++11)
    {
      float w = sqrt(v.dot(v));
      if (w < *config_pallet_.block_width_min || w > *config_pallet_.block_width_max)
      {
        return false;
      }

      std::vector<Eigen::Vector2f> centcentroid;
      Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
      Eigen::Vector2f median = Eigen::Vector2f::Zero();
      int count = 0;

      s = tail = h;
      do
      {
        centcentroid.push_back(s->centroid);
        centroid += s->centroid;
        ++count;
        tail = s;
        if (s == t) break;
      }
      while (s = s->right_pair);

      centroid /= count;

      s = h;
      --count;
      uint32_t rgba_i = rgba[region_count % 20];
      do
      {
        // color the cluster
        pcl::PointXYZRGB q;
        q.x = s->centroid.x();
        q.y = s->centroid.y();
        q.z = 0.0f;
        q.rgba = rgba_i;
        cloud_segmented->points.push_back(q);

        if (count == 0)
        {
          median += s->centroid;
        }
        else if (count == 1 || count == -1)
        {
          median += s->centroid / 2;
        }
        count -= 2;
        if (s == t) break;
      }
      while (s = s->right_pair);

      centcentroids.push_back(centcentroid);
      centroids.push_back(centroid);
      medians.push_back(median);
      minimas.push_back(minima);
      columns.push_back(std::make_pair(h->u, tail->u));
      ++region_count;

      ROS_DEBUG("Minima: (%f %f)", minima.x(), minima.y());

      return true;
    };

    if (!cluster(head, NULL, s->centroid - head->centroid))
    {
      Eigen::Vector2f d = s->centroid - minima;
      cluster(head, tail, minima - head->centroid);
      cluster(tail, NULL, d);
    }
  }

  if (cloud_colored_pub_.getNumSubscribers() > 0)
  {
    cloud_segmented->width = cloud_segmented->points.size();
    cloud_segmented->height = 1;
    cloud_colored_pub_.publish(cloud_segmented);
  }
  ROS_DEBUG("Sizes: %d -> %d -> %d. Segments: %d -> %d (%ld)",
            width * height, voxel_count_, vertical_count_,
            strip_count, region_count, cloud_segmented->points.size());


  // Find centroid pairs and triplets
  std::vector<geometry_msgs::Pose2D> targets;
  std::vector<std::vector<int>> centroid_pairs(centroids.size());
  const float min_sq_dist = *config_pallet_.blocks_distance_min * *config_pallet_.blocks_distance_min;
  const float max_sq_dist = *config_pallet_.blocks_distance_max * *config_pallet_.blocks_distance_max;
  const float cos_alignment_angle = -std::cos(*config_pallet_.blocks_alignment_angle / 180.0 * M_PI);

  cloud_deck_ = make_shared<PointCloud0>();
  cloud_deck_->header.frame_id = "base";

  for (int i = 0; i < centroids.size(); ++i)
  {
    Eigen::Vector2f& ci = minimas[i];

    for (int j = i + 1; j < centroids.size(); ++j)
    {
      Eigen::Vector2f& cj = minimas[j];

      Eigen::Vector2f ij = cj - ci;
      float sq_dist = ij.dot(ij);

      if (sq_dist >= min_sq_dist && sq_dist <= max_sq_dist)
      {
        ROS_DEBUG("C: %f (%f %f) (%f %f)", sqrt(sq_dist), ci.x(), ci.y(), cj.x(), cj.y());

        ij.normalize();
        if (has_first_target_ || fabs(ij.x()) < 0.7071)  // sin(45deg) = 0.7071
        {
          centroid_pairs[i].push_back(j);
          if (*config_pallet_.pocket_count != 1)
          {
            centroid_pairs[j].push_back(i);
          }
          ROS_DEBUG("Paired");
        }
        else
        {
          ROS_DEBUG("Not paired");
        }
      }
    }

    if (centroid_pairs[i].size() < *config_pallet_.pocket_count) continue;

    for (int j = 0; j < centroid_pairs[i].size(); ++j)
    {
      Eigen::Vector2f& cj = minimas[centroid_pairs[i][j]];

      if (*config_pallet_.pocket_count == 1)
      {
        Eigen::Vector2f c, v = ci - cj;
        v.normalize();

        if (*config_pallet_.center_block_line_fitting_max_error)
        {
          fitBlockLine(c, v, cj, centcentroids[i], centcentroids[centroid_pairs[i][j]]);
        }
        else
        {
          c = (ci + cj) * 0.5;
        }

        const auto& column = columns[centroid_pairs[i][j]];
        addTarget(column.first, column.second, c, v, camera_x, targets);
        continue;
      }

      for (int k = j + 1; k < centroid_pairs[i].size(); ++k)
      {
        Eigen::Vector2f& ck = minimas[centroid_pairs[i][k]];

        Eigen::Vector2f ij = (cj - ci).normalized();
        Eigen::Vector2f ik = (ck - ci).normalized();

        float dot = ij.dot(ik);
        if (dot > cos_alignment_angle)
        {
          ROS_DEBUG("Discarded triplets: %f (%f %f) (%f %f) (%f %f)", dot, ci.x(), ci.y(), cj.x(), cj.y(), ck.x(), ck.y());
          continue;
        }

        ROS_DEBUG("Triplets: %f (%f %f) (%f %f) (%f %f)", dot, ci.x(), ci.y(), cj.x(), cj.y(), ck.x(), ck.y());

        Eigen::Vector2f c, v = cj - ck;
        v.normalize();

        if (*config_pallet_.center_block_line_fitting_max_error)
        {
          if (!fitBlockLine(c, v, ck, centcentroids[i], centcentroids[i]))
          {
            ROS_DEBUG("Center block line fitting rejected");
            continue;
          }
        }
        else
        {
          c = (ci + cj + ck) / 3;
        }

        const auto& column_j = columns[centroid_pairs[i][j]];
        const auto& column_k = columns[centroid_pairs[i][k]];
        int column_start = std::min(column_j.first, column_k.first);
        int column_end = std::max(column_j.second, column_k.second);
        addTarget(column_start, column_end, c, v, camera_x, targets);
      }
    }
  }

  if (cloud_deck_pub_.getNumSubscribers() > 0)
  {
    cloud_deck_->width = cloud_deck_->points.size();
    cloud_deck_->height = 1;
    cloud_deck_pub_.publish(cloud_deck_);
  }

  if (targets.empty())
  {
    ROS_DEBUG("No target found");
    return false;
  }
  else if (targets.size() > 1)
  {
    ROS_DEBUG("More than one target found");
    return false;
  }

  sendTarget(targets, depth_voxel_.header.stamp);
  return true;
}

bool PalletDetector::fitBlockLine(Eigen::Vector2f& center, Eigen::Vector2f& gradient, Eigen::Vector2f& c0,
                                  std::vector<Eigen::Vector2f>& cc1, std::vector<Eigen::Vector2f>& cc2)
{
  Eigen::Vector2f vp(-gradient.y(), gradient.x());

  for (int k = cc1.size(); k;)
  {
    Eigen::Vector2f& cc = cc1[--k];
    Eigen::Vector2f ccd = cc - c0;
    float err = gradient.x() * ccd.y() - gradient.y() * ccd.x();
    if (fabs(err) < *config_pallet_.center_block_line_fitting_max_error)
    {
      center = (cc - vp * err);
      break;
    }
    if (!k)
    {
      return false;
    }
  }
  for (auto& cc : cc2)
  {
    Eigen::Vector2f ccd = cc - c0;
    float err = gradient.x() * ccd.y() - gradient.y() * ccd.x();
    if (fabs(err) < *config_pallet_.center_block_line_fitting_max_error)
    {
      center += (cc - vp * err);
      break;
    }
  }
  center *= 0.5;

  return true;
}

bool PalletDetector::fitPalletDeck(int column_start, int column_end, Eigen::Vector2f center, Eigen::Vector2f gradient)
{
  const int width = depth_vertical_.width;
  const int height = depth_vertical_.height;
  const int cloud_row_step = width;
  const float crop_z_min = (*config_pallet_.crop_z_min + *config_pallet_.crop_z_max) / 2;
  float threshold = std::cos(*config_pallet_.block_smoothness_angle / 180.0 * M_PI);
  threshold *= threshold;

  int fit = 0;
  for (int u = column_start; u <= column_end; ++u)
  {
    for (int v = 0; v < height; ++v)
    {
      pcl::PointXYZ& p = cloud_vertical_->points[v * cloud_row_step + u];

      if (isnan(p.x) || p.z < crop_z_min)
      {
        continue;
      }

      Eigen::Vector2f d = p.getVector3fMap().head<2>() - center;
      float err = gradient.x() * d.y() - gradient.y() * d.x();
      if (fabs(err) <= *config_pallet_.deck_line_fitting_max_error)
      {
        ++fit;
        cloud_deck_->points.push_back(p);
        break;
      }
    }
  }

  ROS_DEBUG("Deck fit %d out of %d", fit, column_end - column_start + 1);
  return fit > (column_end - column_start + 1) * threshold;
}

void PalletDetector::addTarget(int column_start, int column_end, Eigen::Vector2f center, Eigen::Vector2f gradient,
                               const float camera_x, std::vector<geometry_msgs::Pose2D>& targets)
{
  if (*config_pallet_.deck_line_fitting_max_error)
  {
    if (!fitPalletDeck(column_start, column_end, center, gradient))
    {
      ROS_DEBUG("Rejected due to missing deck");
      return;
    }
  }

  Eigen::Vector2f c_off(center.x() - camera_x, center.y());
  Eigen::Vector2f vp(-gradient.y(), gradient.x());
  if (vp.dot(c_off) > 0)
  {
    vp = Eigen::Vector2f(gradient.y(), -gradient.x());
  }

  if (!has_first_target_)
  {
    if (*config_pallet_.first_position_max && fabs(center.y()) > *config_pallet_.first_position_max)
    {
      ROS_DEBUG("Rejected due to y-axis offset");
      return;
    }
    if (*config_pallet_.first_orientation_max && fabs(vp.x()) < std::cos(*config_pallet_.first_orientation_max / 180.0 * M_PI))
    {
      ROS_DEBUG("Rejected due to orientation");
      return;
    }
  }

  geometry_msgs::Pose2D target;
  target.x = center.x();
  target.y = center.y();
  target.theta = std::atan2(vp.y(), vp.x());
  targets.push_back(target);
}

void PalletDetector::sendTarget(const std::vector<geometry_msgs::Pose2D>& targets, const ros::Time& timestamp)
{
  std_msgs::Header header;
  header.frame_id = "base";
  header.stamp = timestamp;
  if (!std::isfinite(handleMarkers(header, targets, *config_pallet_.position_deviation_max).x))
  {
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "marker";
  marker.header.stamp = ros::Time();
  marker.ns = "pallet";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -0.45;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.05;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.scale.x = 0.9;
  marker.scale.y = 0.9;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker_viz_pub_.publish(marker);
}


/* static variables */
ros::Publisher PalletDetector::cloud_voxel_pub_;
ros::Publisher PalletDetector::cloud_vertical_pub_;
ros::Publisher PalletDetector::cloud_colored_pub_;
ros::Publisher PalletDetector::cloud_deck_pub_;
ros::Publisher PalletDetector::marker_viz_pub_;
image_transport::Publisher PalletDetector::depth_voxel_pub_;
image_transport::Publisher PalletDetector::depth_vertical_pub_;

}  // namespace marker_localization
