/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Target */
  var marker = scene.append('g');
  marker.attrs({
    class: 'marker',
    display: 'none'
  });
  marker
    .append('polygon')
    .attr(
      'points',
      '-0.01,0.5 0.01,0.5 0.01,0.01 0.2,0.01 0.2,-0.01 0.01,-0.01 0.01,-0.5 -0.01 -0.5'
    );

  var target = scene.append('g');
  target.attrs({
    class: 'target',
    display: 'none'
  });
  target.append('circle').attr('r', 0.04);

  var dockingPlan = target.append('path').attrs({
    class: 'robot-planner-path'
  });
  var dockingPlanAnimation = target.append('path').attrs({
    class: 'robot-planner-path-ani'
  });

  var lastKnownMarkerPose;
  var lastKnownTargetPose;
  var lastKnownPlan;

  function modelsUpdated() {
    updateMarkerPose(lastKnownMarkerPose);
    updateTargetPose(lastKnownTargetPose);
    updateDockingPlan(lastKnownPlan);
  }

  function updateMarkerPose(data) {
    lastKnownMarkerPose = data;
    if (!data) {
      marker.attr('display', 'none');
    } else {
      let deg = (data.theta * 180) / Math.PI;
      marker.attrs({
        transform: `translate(${data.x},${data.y})rotate(${deg})`,
        display: 'block'
      });
    }
  }

  function updateTargetPose(data) {
    lastKnownTargetPose = data;
    if (!data) {
      return;
    }

    var deg = (data.theta * 180) / Math.PI;
    target.attrs({
      transform: `translate(${data.x},${data.y})rotate(${deg})`
    });
  }

  function updateDockingPlan(data) {
    lastKnownPlan = data;
    target.attr('display', 'none');

    if (!data || !data.length) {
      return;
    }

    var d = 'M' + data.join('L');
    dockingPlan.attr('d', d);
    dockingPlanAnimation.attr('d', d);
    target.attr('display', 'block');

    dockingPlanAnimation.style({
      'stroke-dasharray': null,
      'stroke-dashoffset': null
    });
    try {
      let len = dockingPlanAnimation.node().getTotalLength();
      dockingPlanAnimation.style({
        'stroke-dasharray': `0 ${len} ${len} 0`,
        'stroke-dashoffset': len * 2
      });
    } catch (err) {}
  }

  return {
    modelsUpdated: modelsUpdated,
    updateMarkerPose: updateMarkerPose,
    updateTargetPose: updateTargetPose,
    updateDockingPlan: updateDockingPlan
  };
}
