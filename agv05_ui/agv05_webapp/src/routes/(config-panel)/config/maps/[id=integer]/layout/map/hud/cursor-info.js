/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

export default function (viz, hud) {
  /* Cursor Info */
  var cursorInfo = hud
    .append('g')
    .attr('class', 'cursor-info')
    .attr('transform', 'translate(10,-10)');
  var text = cursorInfo.append('text');

  viz.on('mousemove.cursorInfo', function (e) {
    var pt = d3.pointer(e, this);
    var coord = viz.zoom.pixelToCoord(pt);
    coord[0] = coord[0].toFixed(1);
    coord[1] = coord[1].toFixed(1);
    text.text(`x: ${coord[0]}, y: ${coord[1]}`);
  });

  return cursorInfo;
}
