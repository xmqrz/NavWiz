/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import cloudF from './cloud';
import laserScanF from './laser-scan';
import reflectorsF from './reflectors';

export default function (viz, scene) {
  /* Robot */
  var defaultRobotSvg = viz.select('#agv05 path').attr('d');
  var robotPath = scene.append('path').attrs({
    class: 'robot-path',
    display: 'none'
  });
  var plannerPath = scene.append('path').attrs({
    class: 'robot-planner-path',
    display: 'none'
  });
  var plannerPathAnimation = scene.append('path').attrs({
    class: 'robot-planner-path-ani',
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
    transform: 'scale(1,-1)rotate(90)',
    d: defaultRobotSvg
  });

  var robot = scene.append('g').attrs({
    class: 'robot',
    display: 'none'
  });
  var sprite = robot.append('path').attrs({
    transform: 'scale(1,-1)rotate(90)',
    d: defaultRobotSvg
  });
  var payload = robot.append('g').attrs({
    class: 'payload'
  });
  var payloadSprite = payload.append('path').attrs({
    transform: 'scale(1,-1)rotate(90)'
  });
  var laserScan2 = laserScanF(viz, scene, robot, 'laser2');
  var laserScan = laserScanF(viz, scene, robot, 'laser');
  var reflectors2 = reflectorsF(viz, scene, robot, false, 'laser2');
  var reflectors = reflectorsF(viz, scene, robot, false, 'laser');
  var dockingReflectors6 = reflectorsF(viz, scene, robot, true, 'laser6');
  var dockingReflectors5 = reflectorsF(viz, scene, robot, true, 'laser5');
  var dockingReflectors4 = reflectorsF(viz, scene, robot, true, 'laser4');
  var dockingReflectors3 = reflectorsF(viz, scene, robot, true, 'laser3');
  var dockingReflectors2 = reflectorsF(viz, scene, robot, true, 'laser2');
  var dockingReflectors = reflectorsF(viz, scene, robot, true, 'laser');
  var dockingCloud = cloudF(viz, scene, robot);

  var Utils = scene.Utils;

  var lastKnownPose;
  var lastKnownPath;
  var lastKnownMotion;
  var focus = false;

  function modelsUpdated() {
    updateRobotPose(lastKnownPose);
    updateRobotPath(lastKnownPath);
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

    if (!data) {
      return;
    }

    var deg = (data.theta * 180) / Math.PI;
    robot.attrs({
      transform: `translate(${data.x},${data.y})rotate(${deg})`,
      display: 'block'
    });
    if (focus) {
      viz.zoom.recenter([data.x, data.y]);
    }
  }

  function updateRobotPath(data) {
    lastKnownPath = data;
    plannerPath.attr('display', 'none');
    plannerPathAnimation.attr('display', 'none');

    /* update robot path */
    if (!data || !data.length) {
      return;
    }

    var d = 'M' + data.join('L');
    plannerPath.attr('d', d).attr('display', 'block');
    plannerPathAnimation.attr('d', d).attr('display', 'block');

    plannerPathAnimation.styles({
      'stroke-dasharray': null,
      'stroke-dashoffset': null
    });
    try {
      let len = plannerPathAnimation.node().getTotalLength();
      plannerPathAnimation.styles({
        'stroke-dasharray': `0 ${len} ${len} 0`,
        'stroke-dashoffset': len * 2
      });
    } catch (err) {
      // do nothing
    }
  }

  function updateRobotMotion(data) {
    lastKnownMotion = data;
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
    var deg2 = data.location[1];
    robotShadow.attrs({
      transform: `translate(${pt2.x},${pt2.y})rotate(${deg2})`
    });

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
    var deg1 = data.prev_location[1];

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
        if (rot > 0) {
          rot -= 360;
        }
      } else if (data.motion === 'Rotate Right') {
        if (rot < 0) {
          rot += 360;
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

  function updateLaserPose(data) {
    laserScan.updateLaserPose(data);
    laserScan2.updateLaserPose(data);
    reflectors.updateLaserPose(data);
    reflectors2.updateLaserPose(data);
    dockingReflectors.updateLaserPose(data);
    dockingReflectors2.updateLaserPose(data);
    dockingReflectors3.updateLaserPose(data);
    dockingReflectors4.updateLaserPose(data);
    dockingReflectors5.updateLaserPose(data);
    dockingReflectors6.updateLaserPose(data);
  }

  function updateLaserScan(data) {
    laserScan.updateLaserScan(data);
    laserScan2.updateLaserScan(data);
  }

  function updateReflectors(data) {
    reflectors.updateReflectors(data);
    reflectors2.updateReflectors(data);
  }

  function updateDockingReflectors(data) {
    dockingReflectors.updateReflectors(data);
    dockingReflectors2.updateReflectors(data);
    dockingReflectors3.updateReflectors(data);
    dockingReflectors4.updateReflectors(data);
    dockingReflectors5.updateReflectors(data);
    dockingReflectors6.updateReflectors(data);
  }

  return {
    modelsUpdated: modelsUpdated,
    updateRobotSvg: updateRobotSvg,
    updateRobotPose: updateRobotPose,
    updateRobotPath: updateRobotPath,
    updateRobotMotion: updateRobotMotion,
    toggleRobotFocus: toggleRobotFocus,
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    updateReflectors: updateReflectors,
    updateDockingReflectors: updateDockingReflectors,
    updateDockingCloud: dockingCloud.updateCloud
  };
}
