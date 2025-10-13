/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

// manipulate this value in your browser's developer console
// to visualize the effect of changing the threshold
window.lidarIntensityThreshold = 240;
var MAX_SAMPLE = 1440;

export default function (viz, scene, robot, frameId) {
  /* Laser Scan */
  var robotFrame = scene.append('g').attr('class', 'robot-frame');
  var laserScan = robotFrame.append('g').attr('class', `laser-scan ${frameId}`);
  var poly = laserScan.append('path');
  var traces = laserScan.append('path').attr('class', 'traces');

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
    laserScan.attr('transform', transform);
  }

  function updateLaserScan(data) {
    if (frameId !== data.frame_id) {
      return;
    }
    // sync frame transform from actual robot
    robotFrame.attr('transform', robot.attr('transform'));
    var points = [[0, 0]];
    var increment = 1;
    if (viz.options?.laserDownSample) {
      increment = Math.ceil(data.ranges.length / MAX_SAMPLE);
    }
    var intensities;
    if (increment !== 1 && data.intensities) {
      intensities = new Float32Array(data.intensities.length);
    }
    for (let i = 0, j = 0; i < data.ranges.length; i += increment, j++) {
      let radius = data.ranges[i];
      if (isNaN(radius) || radius >= data.range_max) {
        radius = 0;
      }
      let angle = data.angle_min + i * data.angle_increment;
      points.push([radius * Math.cos(angle), radius * Math.sin(angle)]);
      if (increment !== 1 && data.intensities) {
        intensities[j] = data.intensities[i];
      }
    }

    var line = d3.line();
    poly.attr('d', line(points) + 'Z');

    if (viz.zoom.getSnapDistance() <= 0.2) {
      traces.attr('d', pointTraces(points, increment !== 1 ? intensities : data.intensities));

      traces.style('display', 'block');
    } else {
      traces.style('display', 'none');
    }
  }

  function pointTraces(points, intensities) {
    return points
      .map((d, i) => {
        let r = 0.002;
        if (intensities) {
          var thres = window.lidarIntensityThreshold;
          r = intensities[i] >= thres ? 0.006 : 0.002;
        }
        return `M ${d[0]} ${d[1]} m ${-r} 0 a ${r},${r} 0 1,0 ${r + r},0 a ${r},${r} 0 1,0 -${r + r},0`;
      })
      .join(' ');
  }

  function clearLaserScan() {
    poly.attr('d', '');
    traces.style('display', 'none');
  }

  return {
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    clearLaserScan: clearLaserScan
  };
}
