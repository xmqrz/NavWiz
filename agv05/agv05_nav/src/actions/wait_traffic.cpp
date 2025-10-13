/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

#include "agv05_nav/actions.h"


namespace agv05
{

void ActionWaitTraffic::process(float frequency)
{
  if (isCompleted()) return;

  publishStatus(Feedback::STATUS_WAIT_TRAFFIC);

  if (isAborted())
  {
    completeNav(Result::RESULT_SUCCESS);
  }
}

}  // namespace agv05
