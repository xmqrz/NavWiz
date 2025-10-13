/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene, robot, docking, frameId) {
  /* Reflectors */
  docking = docking ? ' docking' : '';
  var robotFrame = scene.append('g').attr('class', 'robot-frame');
  var reflectors = robotFrame.append('g').attr('class', `reflectors${docking} ${frameId}`);

  function updateLaserPose(data) {
    if (!(frameId in data)) {
      return;
    }
    var pose = data[frameId];
    var deg = (pose.theta * 180) / Math.PI;
    var transform = `translate(${pose.x},${pose.y})rotate(${deg})`;
    if (pose.flip) {
      transform += 'scale(1,-1)';
    }
    reflectors.attr('transform', transform);
  }

  function updateReflectors(data) {
    if (frameId !== data.frame_id) {
      if (docking) {
        reflectors.attr('display', 'none');
      }
      return;
    }
    // sync frame transform from actual robot
    robotFrame.attr('transform', robot.attr('transform'));

    var r = reflectors.selectAll('use').data(data.points).join('use');
    r.attrs({
      class: `reflector${docking}`,
      'xlink:href': '#landmark',
      x: (d) => d[0],
      y: (d) => d[1]
    });

    if (docking) {
      reflectors.attr('display', 'block');
    }
  }

  return {
    updateLaserPose: updateLaserPose,
    updateReflectors: updateReflectors
  };
}
