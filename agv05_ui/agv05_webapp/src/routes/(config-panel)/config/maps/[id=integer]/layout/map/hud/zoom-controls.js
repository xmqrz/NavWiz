/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, hud) {
  /* Zoom Controls */
  var zoomControls = hud.append('g').attr('class', 'zoom-controls');
  var zoomIn = zoomControls
    .append('use')
    .attr('xlink:href', '#zoom-in')
    .attr('tip-title', 'Zoom in')
    .attr('x', 26)
    .attr('y', -90);
  var zoomOut = zoomControls
    .append('use')
    .attr('xlink:href', '#zoom-out')
    .attr('tip-title', 'Zoom out')
    .attr('x', 26)
    .attr('y', -60);

  zoomIn.on('click', function (event) {
    event.stopPropagation();
    viz.zoom.zoomIn();
  });
  zoomIn.on('dblclick', function (event) {
    event.stopPropagation();
  });
  zoomOut.on('click', function (event) {
    event.stopPropagation();
    viz.zoom.zoomOut();
  });
  zoomOut.on('dblclick', function (event) {
    event.stopPropagation();
  });

  return zoomControls;
}
