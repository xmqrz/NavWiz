/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene, robot) {
  /* Cloud */
  var robotFrame = scene.append('g').attr('class', 'robot-frame');
  var cloud = robotFrame.append('g').attr('class', 'cloud docking');

  function updateCloud(data) {
    if (!data.frame_id) {
      cloud.attr('display', 'none');
    }

    // sync frame transform from actual robot
    robotFrame.attr('transform', robot.attr('transform'));
    var points = [];
    for (let i = 0; i < data.cloud.length; i += 4) {
      points.push([data.cloud[i], data.cloud[i + 1]]);
    }

    var p = cloud.selectAll('circle').data(points).join('circle');
    p.attrs({
      r: 0.01,
      cx: (d) => d[0],
      cy: (d) => d[1]
    });

    cloud.attr('display', 'block');
  }

  return {
    updateCloud: updateCloud
  };
}
