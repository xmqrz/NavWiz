/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Authors: Patrick Chin
 */

#include "agv05_navx/nav.h"

#include <agv05_navx/action_processor.h>


namespace agv05
{

Nav::Nav() :
  as_(nh_, "navx_action", boost::bind(&Nav::handleGoal, this, _1), false),
  base_(nh_),
  io_(nh_),
  laser_sensor_(nh_),
  line_sensor_(nh_),
  panel_(nh_),
  safety_(nh_),
  diagnostic_(nh_, base_, config_)
{
  // action server
  as_.start();

  // dynamic reconfigure server
  config_.setupProfileDefault();
  ds_.setCallback(boost::bind(&Nav::handleConfig, this, _1, _2));

  // subscriber
  nav_control_sub_ = nh_.subscribe("agv05/navx/navx_control", 10, &Nav::handleNavControl, this);

  ROS_INFO_STREAM("agv05_navx started.");
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

  pose_.x = 0;
  pose_.y = 0;
  pose_.theta = 0;
  base_.getPose(pose_);

  if (processor_)
  {
    processor_->process(frequency);
    diagnostic_.update(processor_->getActionType(), processor_->getStatus(), pose_, processor_->getOvershoot(), processor_->getLinearErr(),
                       processor_->getAngularErr(), processor_->getHeadingErr());
  }
  else
  {
    ActionProcessor::process(frequency, *this);
    // linear error, angular error and heading error are only valid during motion
    diagnostic_.update(Goal::NAV_IDLE, Feedback::STATUS_NORMAL, pose_, 0.0f, 0.0f, 0.0f);
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
