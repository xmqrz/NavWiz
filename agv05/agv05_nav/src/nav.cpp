/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: phtan
 */

#include "agv05_nav/nav.h"

#include <agv05_nav/action_processor.h>


namespace agv05
{

Nav::Nav() :
  as_(nh_, "nav_action", boost::bind(&Nav::handleGoal, this, _1), false),
  base_(nh_),
  io_(nh_),
  laser_sensor_(nh_),
  line_sensor_(nh_),
  panel_(nh_),
  safety_(nh_),
  diagnostic_(nh_, config_)
{
  // action server
  as_.start();

  // dynamic reconfigure server
  config_.setupProfileDefault();
  ds_.setCallback(boost::bind(&Nav::handleConfig, this, _1, _2));

  // subscriber
  nav_control_sub_ = nh_.subscribe("agv05/nav/nav_control", 10, &Nav::handleNavControl, this);

  ROS_INFO_STREAM("agv05_nav started.");
}

Nav::~Nav()
{
  // explicitly call shutdown to make sure the execute thread (handleGoal)
  // has exited before the class is destructed.
  as_.shutdown();
}

void Nav::process(float frequency)
{
  Lock lock(mutex_);

  if (processor_)
  {
    processor_->process(frequency);
    diagnostic_.update(processor_->getActionType(), processor_->getStatus(), processor_->getOvershoot());
  }
  else
  {
    ActionProcessor::process(frequency, *this);
    diagnostic_.update(Goal::NAV_IDLE, Feedback::STATUS_NORMAL);
  }

  safety_.publishSafetyHeartbeat();
}

void Nav::handleGoal(const GoalConstPtr& goal)
{
  ROS_INFO("Action received: nav %d speed %f enable_sensor %d next_motion %d", goal->nav, goal->speed, (uint8_t)goal->enable_sensor, goal->next_motion);

  Lock lock(mutex_);

  ROS_ASSERT_MSG(!processor_, "The current goal is still active.");

  /* create action processor */
  processor_ = ActionProcessor::create(*this, *goal);
  if (!processor_)
  {
    // goal validation error
    as_.setAborted();
    return;
  }

  ros::Rate rate(10);
  Feedback feedback;

  while (ros::ok())
  {
    // update status
    if (processor_->isStatusUpdated())
    {
      processor_->ackStatusUpdate();
      feedback.status = processor_->getStatus();
      as_.publishFeedback(feedback);
    }
    // check if nav completed
    if (processor_->isCompleted())
    {
      break;
    }
    // release lock while sleeping
    Unlock unlock(lock);
    rate.sleep();
  }

  Result result;
  result.result = processor_->getResult();
  as_.setSucceeded(result);

  /* destroy action processor */
  processor_.reset();
}

void Nav::handleNavControl(const NavControl& nav_control)
{
  Lock lock(mutex_);
  if (processor_)
  {
    processor_->handleNavControl(nav_control);
  }
}

}  // namespace agv05
