/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import robotF from './robot';

export default function (viz, scene) {
  var liveScene = scene.append('g').attr('class', 'agv05-live-scene');
  liveScene.Utils = scene.Utils;
  var robot = robotF(viz, liveScene);

  function show() {
    liveScene.classed('hidden', false);
  }

  function hide() {
    robot.updateRobotPose();
    liveScene.classed('hidden', true);
  }

  hide();
  return {
    show: show,
    hide: hide,
    updateRobotSvg: robot.updateRobotSvg,
    updateRobotPose: robot.updateRobotPose,
    updateLaserPose: robot.updateLaserPose,
    updateLaserScan: robot.updateLaserScan,
    updateReflectors: robot.updateReflectors
  };
}
