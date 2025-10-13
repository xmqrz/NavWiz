/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_navx/actions.h"


namespace agv05
{

ActionSelectNavProfile::ActionSelectNavProfile(Nav& nav, const Goal& goal) :
  ActionProcessor(nav, goal)
{
  uint8_t profile = goal.nav - Goal::NAV_SELECT_NAV_PROFILE1;
  ROS_ASSERT(profile <= Goal::NAV_SELECT_NAV_PROFILE5 - Goal::NAV_SELECT_NAV_PROFILE1);

  config_.selectProfile(profile);
  completeAction(Result::RESULT_SUCCESS);
}

}  // namespace agv05
