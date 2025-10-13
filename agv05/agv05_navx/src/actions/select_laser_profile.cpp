/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"


namespace agv05
{

ActionSelectLaserProfile::ActionSelectLaserProfile(Nav& nav, const Goal& goal) :
  ActionProcessor(nav, goal)
{
  uint8_t profile = goal.nav - Goal::NAV_SELECT_LASER_PROFILE1;
  ROS_ASSERT(profile <= Goal::NAV_SELECT_LASER_PROFILE10 - Goal::NAV_SELECT_LASER_PROFILE1);

  laser_sensor_.selectProfile(profile + 1);
  completeAction(Result::RESULT_SUCCESS);
}

}  // namespace agv05
