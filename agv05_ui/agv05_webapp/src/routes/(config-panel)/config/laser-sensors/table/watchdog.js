/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default function (inspector, frameId) {
  var pose;
  var detected = false;

  function updateLaserPose(data) {
    if (!(frameId in data)) {
      return;
    }
    pose = data[frameId];

    //reset detection
    detected = false;
  }

  function updateLaserScan(data) {
    if (frameId !== data.frame_id) {
      return;
    }
    if (!pose) {
      return;
    }

    let points = [];
    for (let i = 0; i < data.ranges.length; i++) {
      let radius = data.ranges[i];
      if (isNaN(radius) || radius <= 0 || radius > data.range_max) {
        continue;
      }
      let angle = data.angle_min + i * data.angle_increment;
      let point = [radius * Math.cos(angle), radius * Math.sin(angle)];
      if (pose.flip) {
        point[1] = -point[1];
      }
      if (pose.theta) {
        let x = point[0];
        let y = point[1];
        point[0] = x * Math.cos(pose.theta) - y * Math.sin(pose.theta);
        point[1] = x * Math.sin(pose.theta) + y * Math.cos(pose.theta);
      }
      point[0] += pose.x;
      point[1] += pose.y;
      // convert to mm
      point[0] *= 1000;
      point[1] *= 1000;
      points.push(point);
    }

    detected = checkInsideRegion(points);
  }

  function checkInsideRegion(points) {
    let min_activation = inspector.getMinActivation();
    let region = inspector.getRegion();
    let regionParameter = inspector.getRegionParameter();
    let minX = regionParameter[0];
    let minY = regionParameter[1];
    let maxX = regionParameter[2];
    let maxY = regionParameter[3];

    let activation = 0;
    for (let i in points) {
      let pt = points[i];
      let inside = false;
      if (pt[0] < minX || pt[0] > maxX || pt[1] < minY || pt[1] > maxY) {
        continue;
      }
      for (let i = 0, j = region.length - 1; i < region.length; j = i++) {
        // vertex hit
        if (region[i][0] === pt[0] && region[i][1] === pt[1]) {
          inside = true;
          break;
        }

        // parallel border hit
        if (
          region[i][1] === pt[1] &&
          region[j][1] === pt[1] &&
          region[i][0] > pt[0] !== region[j][0] > pt[0]
        ) {
          inside = true;
          break;
        }

        if (region[i][1] > pt[1] !== region[j][1] > pt[1]) {
          let xIntercept =
            ((region[j][0] - region[i][0]) * (pt[1] - region[i][1])) /
              (region[j][1] - region[i][1]) +
            region[i][0];

          // border hit
          if (pt[0] === xIntercept) {
            inside = true;
            break;
          } else if (pt[0] < xIntercept) {
            inside = !inside;
          }
        }
      }
      if (inside) {
        activation++;
      }

      if (activation >= min_activation) {
        break;
      }
    }
    return activation >= min_activation;
  }

  function reset() {
    detected = false;
  }

  return {
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    reset: reset,
    getDetected: () => detected
  };
}
