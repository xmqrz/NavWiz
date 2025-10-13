/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class Metadata {
  constructor(options) {
    options = options || {};
    this.frame_id = options.frame_id || 'map';
    this.resolution = _round3(Math.abs(parseFloat(options.resolution)) || 0.05);
    this.width = Math.abs(parseInt(options.width)) || 10;
    this.height = Math.abs(parseInt(options.height)) || 10;
    this.x0 = _round3(parseFloat(options.x0) || 0);
    this.y0 = _round3(parseFloat(options.y0) || 0);

    // temporary variables when resizing canvas
    this.x_shift = 0;
    this.y_shift = 0;
    this.png = null;
  }
}

Metadata.__attrs__ = ['frame_id', 'resolution', 'width', 'height', 'x0', 'y0'];

/* Round to 3 decimals */
function _round3(v) {
  return Math.round(v * 1000) / 1000;
}
