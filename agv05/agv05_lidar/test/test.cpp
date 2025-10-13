/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "agv05_lidar/lidar_watchdog.h"

/* Front Lidar Test */

TEST(WatchdogTransformTest, lidarFrontRangeStraightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 0.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 2.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 0.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, lidarFrontRangeRightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = -3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, lidarFrontRangeLeftTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, lidarFrontRangeBackTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 0.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 0.0f, 1.0e-3);
};

/* Front Left Lidar Test */

TEST(WatchdogTransformTest, lidarLeftRangeStraightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 0.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, lidarLeftRangeRightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = -3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, lidarLeftRangeLeftTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, lidarLeftRangeBackTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(0.0f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

/* Inverted Front Lidar Test */

TEST(WatchdogTransformTest, invertedLidarFrontRangeStraightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 0.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 2.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 0.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarFrontRangeRightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = -3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarFrontRangeLeftTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f, 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarFrontRangeBackTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.0f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 0.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 0.0f, 1.0e-3);
  ASSERT_NEAR(pt.imag(), 0.0f, 1.0e-3);
};

/* Front Inverted Left Lidar Test */

TEST(WatchdogTransformTest, invertedLidarLeftRangeStraightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 0.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarLeftRangeRightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = -3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarLeftRangeLeftTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarLeftRangeBackTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 0.785f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, 1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), 1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

/* Front Inverted Right Lidar Test */

TEST(WatchdogTransformTest, invertedLidarRightRangeStraightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 5.497f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, -1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 0.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarRightRangeRightTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 5.497f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, -1.0f, 0.0f));
  float radius = 1.0f;
  float theta = -3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f + 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarRightRangeLeftTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 5.497f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, -1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f / 2.0f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f - 1.0f * std::cos(0.785f), 1.0e-3);
};

TEST(WatchdogTransformTest, invertedLidarRightRangeBackTest)
{
  Watchdog wd = Watchdog("test", "test");
  tf::StampedTransform transform;
  tf::Quaternion qt;
  qt.setRPY(3.14159f, 0.0f, 5.497f);
  transform.setRotation(qt);
  transform.setOrigin(tf::Vector3(1.0f, -1.0f, 0.0f));
  float radius = 1.0f;
  float theta = 3.14159f;

  complex pt = wd.transformPolarToBase(radius, theta, transform);

  ASSERT_NEAR(pt.real(), 1.0f - 1.0f * std::sin(0.785f), 1.0e-3);
  ASSERT_NEAR(pt.imag(), -1.0f + 1.0f * std::cos(0.785f), 1.0e-3);
};


/* Watchdog Hit Test  */

TEST(WatchdogHitTest, triangleTest)
{
  Watchdog wd = Watchdog("test", "test");
  // Triangle region
  std::vector<complex> region;
  region.push_back(complex(0.0f, -1.0f));
  region.push_back(complex(1.0f, 0.0f));
  region.push_back(complex(0.0f, 1.0f));
  complex min(0.0f, -1.0f), max(1.0f, 1.0f), pt;

  // test out bound
  pt.real(1.0f);
  pt.imag(1.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  pt.real(1.0f);
  pt.imag(-1.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  // vertex intercept
  pt.real(0.0f);
  pt.imag(-1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(1.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(0.0f);
  pt.imag(1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  // border intercept
  pt.real(0.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.5f);
  pt.imag(-0.5f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.5f);
  pt.imag(0.5f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  // inside border
  pt.real(0.5f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.9f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.2f);
  pt.imag(0.1f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  // outside border
  pt.real(0.501f);
  pt.imag(0.5f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";

  pt.real(0.501f);
  pt.imag(-0.5f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";
};

TEST(WatchdogHitTest, invertedTriangleTest)
{
  Watchdog wd = Watchdog("test", "test");
  // Triangle region
  std::vector<complex> region;
  region.push_back(complex(1.0f, -1.0f));
  region.push_back(complex(0.0f, 0.0f));
  region.push_back(complex(1.0f, 1.0f));
  complex min(0.0f, -1.0f), max(1.0f, 1.0f), pt;

  // test out bound
  pt.real(0.0f);
  pt.imag(1.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  pt.real(0.0f);
  pt.imag(-1.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  // vertex intercept
  pt.real(1.0f);
  pt.imag(-1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(0.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(1.0f);
  pt.imag(1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  // border intercept
  pt.real(1.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.5f);
  pt.imag(-0.5f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.5f);
  pt.imag(0.5f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  // inside border
  pt.real(0.5f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.9f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.8f);
  pt.imag(0.1f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  // outside border
  pt.real(0.499f);
  pt.imag(0.5f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";

  pt.real(0.499f);
  pt.imag(-0.5f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";
};

TEST(WatchdogHitTest, boxTest)
{
  Watchdog wd = Watchdog("test", "test");
  // box region
  std::vector<complex> region;
  region.push_back(complex(1.0f, -1.0f));
  region.push_back(complex(-1.0f, -1.0f));
  region.push_back(complex(-1.0f, 1.0f));
  region.push_back(complex(1.0f, 1.0f));
  complex min(-1.0f, -1.0f), max(1.0f, 1.0f), pt;

  // test out bound
  pt.real(1.001f);
  pt.imag(1.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  pt.real(0.0f);
  pt.imag(-1.001f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test out of bound";

  // vertex intercept
  pt.real(1.0f);
  pt.imag(-1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(-1.0f);
  pt.imag(-1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(-1.0f);
  pt.imag(1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  pt.real(1.0f);
  pt.imag(1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept vertex";

  // border intercept
  pt.real(1.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(-1.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.0f);
  pt.imag(1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  pt.real(0.0f);
  pt.imag(-1.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test intercept border";

  // inside border
  pt.real(0.0f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.9f);
  pt.imag(0.0f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.8f);
  pt.imag(0.1f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  pt.real(0.5f);
  pt.imag(0.5f);
  ASSERT_TRUE(wd.computeHit(region, min, max, pt)) << "Test inside border";

  // outside border
  pt.real(1.001f);
  pt.imag(0.0f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";

  pt.real(0.0f);
  pt.imag(-1.001f);
  ASSERT_FALSE(wd.computeHit(region, min, max, pt)) << "Test outside border";
};

TEST(LidarInspectorTest, obtainParamTest)
{
  LidarInspector li = LidarInspector();
  ros::NodeHandle private_nh("~");
  ros::NodeHandle fake_private_nh("/fake_namespace");

  // Validate Result
  dict param;
  ASSERT_TRUE(li.obtainParam(private_nh, "primary_obstacle_scan_topics", param));
  ASSERT_EQ(2, param.size());

  dict param2;
  ASSERT_TRUE(li.obtainParam(private_nh, "secondary_obstacle_scan_topics", param2));
  ASSERT_EQ(3, param2.size());

  dict param3;
  ASSERT_FALSE(li.obtainParam(private_nh, "tertiary_obstacle_scan_topics", param3));
  ASSERT_EQ(0, param3.size());

  dict param4;
  ASSERT_TRUE(li.obtainParam(private_nh, "fourth_obstacle_scan_topics", param4));
  ASSERT_EQ(4, param4.size());

  dict param5;
  ASSERT_FALSE(li.obtainParam(fake_private_nh, "primary_obstacle_scan_topics", param5));
  ASSERT_EQ(0, param5.size());

  // Validate Value
  ASSERT_TRUE(param.find("scan") != param.end());
  ASSERT_TRUE(param.find("scan2") != param.end());
  ASSERT_TRUE(param.find("scan3") == param.end());
  ASSERT_EQ(param["scan"], "Laser Primary Front");
  ASSERT_EQ(param["scan2"], "Laser Secondary Back");

  ASSERT_TRUE(param2.find("scan") == param2.end());
  ASSERT_TRUE(param2.find("scan2") != param2.end());
  ASSERT_TRUE(param2.find("scan3") != param2.end());
  ASSERT_TRUE(param2.find("scan4") != param2.end());
  ASSERT_EQ(param2["scan2"], "scan2");
  ASSERT_EQ(param2["scan3"], "scan3");
  ASSERT_EQ(param2["scan4"], "scan4");

  ASSERT_TRUE(param4.find("scan") == param4.end());
  ASSERT_TRUE(param4.find("hokuyo/scan") != param4.end());
  ASSERT_TRUE(param4.find("hokuyo/scan2") != param4.end());
  ASSERT_TRUE(param4.find("sick/scan3") != param4.end());
  ASSERT_TRUE(param4.find("scan4") != param4.end());
  ASSERT_EQ(param4["hokuyo/scan"], "Laser Primary Front");
  ASSERT_EQ(param4["hokuyo/scan2"], "Laser Secondary Back");
  ASSERT_EQ(param4["sick/scan3"], "Rear Hidden Lidar");
  ASSERT_EQ(param4["scan4"], "Main Top Lidar");
};


/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_cpp");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
