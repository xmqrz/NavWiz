/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

export default function (viz, hud) {
  /* AGV Controls */
  var agvControls = hud.append('g').attr('class', 'agv-controls');
  var focusToggle = agvControls
    .append('use')
    .attr('xlink:href', '#focus-toggle')
    .attr('tip-title', 'Toggle focus display')
    .attr('x', 26)
    .attr('y', -120)
    .attr('color', 'lightgray');

  var focus = false;

  focusToggle.on('click', function (event) {
    event.stopPropagation();
    viz.scene.toggleRobotFocus();
    focus = !focus;
    focusToggle.attr('color', focus ? null : 'lightgray');
  });

  return agvControls;
}
