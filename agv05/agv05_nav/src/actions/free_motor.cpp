/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/actions.h"


namespace agv05
{

ActionFreeMotor::ActionFreeMotor(Nav& nav, const Goal& goal) :
  ActionProcessor(nav, goal)
{
  ROS_ASSERT(goal.nav == Goal::NAV_FREE_MOTOR);

  publishStatus(Feedback::STATUS_NORMAL);
  safety_.publishNavTrigger(true);

  // Hack: Use the smallest positive denormal to signal free motor
  geometry_msgs::Twist v;
  *reinterpret_cast<uint64_t*>(&v.linear.z) = 1LL;
  base_.setSpeed(v);
}

void ActionFreeMotor::process(float frequency)
{
  if (isCompleted()) return;

  if (isAborted())
  {
    base_.stop();
    completeNav(Result::RESULT_SUCCESS);
  }
}

}  // namespace agv05
