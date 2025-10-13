/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Robot */
  var defaultRobotSvg = viz.select('#agv05 path').attr('d');
  var robotPath = scene.append('path').attrs({
    class: 'robot-path',
    display: 'none'
  });
  var robotShadow = scene.append('g').attrs({
    class: 'robot-shadow'
  });
  var robotRotation = robotShadow.append('g').attrs({
    class: 'robot-rotation',
    display: 'none'
  });
  var spriteRotation = robotRotation.append('path').attrs({
    d: defaultRobotSvg
  });

  var robot = scene.append('g').attrs({
    class: 'robot',
    display: 'none'
  });
  var sprite = robot.append('path').attrs({
    d: defaultRobotSvg
  });
  var payload = robot.append('g').attrs({
    class: 'payload'
  });
  var payloadSprite = payload.append('path');

  var Utils = scene.Utils;

  var lastKnownPose;
  var lastKnownMotion;
  var focus = false;

  function modelsUpdated() {
    updateRobotPose(lastKnownPose);
    updateRobotMotion(lastKnownMotion);
  }

  function updateRobotSvg(data) {
    spriteRotation.attr('d', data.body);
    sprite.attr('d', data.body);
    payloadSprite.attr('d', data.payload);
  }

  function updateRobotPose(data) {
    lastKnownPose = data;
    robot.attr('display', 'none');

    if (!data || !lastKnownMotion || !lastKnownMotion.location) {
      return;
    }

    var deg = (data.theta * 180) / Math.PI + 90;
    robot.attrs({
      transform: `translate(${data.x},${data.y})rotate(${deg})`,
      display: 'block'
    });
    if (focus) {
      viz.zoom.recenter([data.x, data.y]);
    }
  }

  function updateRobotMotion(data) {
    lastKnownMotion = data;
    robot.attr('display', 'none');
    robotPath.attr('display', 'none');
    robotRotation.attr('display', 'none');

    /* update robot */
    if (!data || !data.location) {
      return;
    }

    var mapIdx = Math.floor(data.location[0] / 100000);

    var junctions = viz.models.rawJunctions();
    if (!junctions) {
      return;
    }

    var j2 = data.location[0] % 100000;
    var pt2 = junctions[j2];
    if (!pt2) {
      return;
    }
    var deg2 = data.location[1] * 90.0;
    var deg = lastKnownPose ? (lastKnownPose.theta * 180) / Math.PI + 90 : deg2;
    var pose = lastKnownPose || pt2;
    robot.attrs({
      transform: `translate(${pose.x},${pose.y})rotate(${deg})`,
      display: 'block'
    });
    robotShadow.attrs({
      transform: `translate(${pt2.x},${pt2.y})rotate(${deg2})`
    });
    if (focus) {
      viz.zoom.recenter([pose.x, pose.y]);
    }

    /* update robot motion */
    if (!data.prev_location || !data.motion) {
      return;
    }

    if (Math.floor(data.prev_location[0] / 100000) !== mapIdx) {
      return;
    }

    var j1 = data.prev_location[0] % 100000;
    var pt1 = junctions[j1];
    if (!pt1) {
      return;
    }
    var deg1 = data.prev_location[1] * 90.0;

    if (pt1 !== pt2) {
      var paths = viz.models.rawPaths();
      if (!paths) {
        return;
      }

      for (let path of paths) {
        if (path.j1 === j1 && path.j2 === j2) {
          robotPath.attrs({
            d: Utils.generateConnectorPath(path, pt1, pt2, true),
            display: 'block'
          });
          robotPath.styles({
            'stroke-dasharray': null,
            'stroke-dashoffset': null
          });
          try {
            let len = robotPath.node().getTotalLength();
            robotPath.styles({
              'stroke-dasharray': `0 ${len} ${len} 0`,
              'stroke-dashoffset': len * 2
            });
          } catch (err) {
            // do nothing
          }
          break;
        } else if (path.j1 === j2 && path.j2 === j1) {
          robotPath.attrs({
            d: Utils.generateConnectorPath(path, pt2, pt1, true),
            display: 'block'
          });
          robotPath.styles({
            'stroke-dasharray': null,
            'stroke-dashoffset': null
          });
          try {
            let len = robotPath.node().getTotalLength();
            robotPath.styles({
              'stroke-dasharray': `0 ${len} ${len} 0`,
              'stroke-dashoffset': -len * 2
            });
          } catch (err) {
            // do nothing
          }
          break;
        }
      }
    } else if (deg1 !== deg2) {
      var rot = deg1 - deg2;
      if (data.motion === 'Rotate Left') {
        if (rot < 0) {
          rot += 360;
        }
      } else if (data.motion === 'Rotate Right') {
        if (rot > 0) {
          rot -= 360;
        }
      } else {
        rot = 0;
      }
      if (rot) {
        robotRotation.attr('display', 'block');
        robotRotation.style('transform', `rotate(${rot}deg)`);
      }
    }
  }

  function toggleRobotFocus() {
    focus = !focus;
    if (focus) {
      viz.zoom.disablePan();
    } else {
      viz.zoom.enable();
    }
  }

  return {
    modelsUpdated: modelsUpdated,
    updateRobotSvg: updateRobotSvg,
    updateRobotPose: updateRobotPose,
    updateRobotMotion: updateRobotMotion,
    toggleRobotFocus: toggleRobotFocus
  };
}
