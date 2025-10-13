/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import laserScanF from 'mapx-layout-editor/scene/laser-scan';

export default function (viz, scene) {
  /* Robot */
  var defaultRobotSvg = viz.select('#agv05 path').attr('d');
  var robot = scene.append('g').attrs({
    class: 'robot',
    display: 'none'
  });
  var sprite = robot.append('path').attrs({
    transform: 'scale(1,-1)rotate(90)',
    d: defaultRobotSvg
  });
  var subScene = scene.append('g').attrs({
    class: 'laser-area-lidar'
  });
  var laserScans = {};

  function updateRobotSvg(data) {
    sprite.attr('d', data.body);
  }

  function updateRobotPose(data) {
    robot.attr('display', 'none');

    if (!data) {
      return;
    }

    var deg = (data.theta * 180) / Math.PI;
    robot.attrs({
      transform: `translate(${data.x},${data.y})rotate(${deg})`,
      display: 'block'
    });
  }

  function updateLaserPose(data) {
    for (let frameId in data) {
      if (!(frameId in laserScans)) {
        laserScans[frameId] = laserScanF(viz, subScene, robot, frameId);
      }
      laserScans[frameId].updateLaserPose(data);
    }
  }

  function updateLaserScan(data) {
    for (let frameId in laserScans) {
      laserScans[frameId].updateLaserScan(data);
    }
  }

  function clearLaserScan() {
    for (let frameId in laserScans) {
      laserScans[frameId].clearLaserScan();
    }
  }

  return {
    updateRobotSvg: updateRobotSvg,
    updateRobotPose: updateRobotPose,
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    clearLaserScan: clearLaserScan
  };
}
