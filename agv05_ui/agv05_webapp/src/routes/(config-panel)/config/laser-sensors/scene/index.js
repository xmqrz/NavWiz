/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import gridF from 'mapx-layout-editor/scene/grid';

import robotF from './robot';
import regionsF from './regions';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  var grid = gridF(viz, scene);
  var robot = robotF(viz, scene);
  var regions = regionsF(viz, scene);
  var curScanId = ['scan'];
  robot.updateRobotPose({
    x: 0,
    y: 0,
    theta: 0
  });

  scene.zoomed = function () {
    scene.attr(
      'transform',
      `translate(${viz.meta.translate})scale(${viz.meta.scale})scale(1,-1)rotate(90)`
    );
    grid.zoomed();
    regions.zoomed();
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    regions.modelsUpdated();
    modelsUpdated();
  };

  scene.setActiveRegion = regions.setActiveRegion;
  scene.setActiveCoord = regions.setActiveCoord;

  function modelsUpdated() {
    curScanId = viz.models.rawScanId();
    robot.clearLaserScan();
  }

  scene.updateRobotSvg = robot.updateRobotSvg;

  scene.updateLaserPose = function (data) {
    if (!data) {
      return;
    }
    robot.updateLaserPose(data);
  };

  scene.updateLaserScan = function (data) {
    if (!curScanId.includes(data.scan_id)) {
      return;
    }

    robot.updateLaserScan(data);
  };

  return scene;
}
