/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, hud) {
  /* AGV Controls */
  var agvControls = hud.append('g').attr('class', 'agv-controls');
  var costmapToggle = agvControls.append('use').attrs({
    'xlink:href': '#costmap-toggle',
    'tip-title': 'Toggle costmap display',
    x: 26,
    y: -150,
    color: 'lightgray'
  });
  var focusToggle = agvControls.append('use').attrs({
    'xlink:href': '#focus-toggle',
    'tip-title': 'Toggle focus display',
    x: 26,
    y: -120,
    color: 'lightgray'
  });

  var costmapShown = false;
  var focus = false;

  costmapToggle.on('click', function (event) {
    event.stopPropagation();
    viz.scene.toggleCostmap();
    costmapShown = !costmapShown;
    costmapToggle.attr('color', costmapShown ? null : 'lightgray');
  });
  focusToggle.on('click', function (event) {
    event.stopPropagation();
    viz.scene.toggleRobotFocus();
    focus = !focus;
    focusToggle.attr('color', focus ? null : 'lightgray');
  });

  return agvControls;
}
