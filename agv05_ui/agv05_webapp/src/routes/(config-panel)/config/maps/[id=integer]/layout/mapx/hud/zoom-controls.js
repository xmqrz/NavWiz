/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, hud) {
  /* Zoom Controls */
  var zoomControls = hud.append('g').attr('class', 'zoom-controls');
  var zoomIn = zoomControls.append('use').attrs({
    'xlink:href': '#zoom-in',
    'tip-title': 'Zoom in',
    x: 26,
    y: -90
  });
  var zoomOut = zoomControls.append('use').attrs({
    'xlink:href': '#zoom-out',
    'tip-title': 'Zoom out',
    x: 26,
    y: -60
  });

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
