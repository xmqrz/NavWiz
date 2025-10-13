/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/actions.h"

#define SCAN_DELAY 1.5f


namespace agv05
{

ActionScanLaser::ActionScanLaser(Nav& nav, const Goal& goal) :
  ActionProcessor(nav, goal),
  delay_(0.0f)
{
  uint8_t area = goal.nav - Goal::NAV_SCAN_LASER_AREA1;
  ROS_ASSERT(area <= Goal::NAV_SCAN_LASER_AREA31 - Goal::NAV_SCAN_LASER_AREA1);

  laser_sensor_.selectArea(area + 1, 0);
  publishStatus(Feedback::STATUS_NORMAL);
}

void ActionScanLaser::process(float frequency)
{
  if (isCompleted()) return;

  delay_ += 1.0f / frequency;

  if (delay_ > SCAN_DELAY)
  {
    if (laser_sensor_.getActivation().near_blocked ||
        laser_sensor_.getActivation().middle_blocked ||
        laser_sensor_.getActivation().far_blocked)
    {
      completeNav(Result::RESULT_SUCCESS);
    }
    else
    {
      completeNav(Result::RESULT_FAIL);
    }
  }
}

}  // namespace agv05
