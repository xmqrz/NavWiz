/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class Junction {
  constructor(options) {
    options = options || {};
    this.layer = options.layer || 0;
    this.x = options.x || 0;
    this.y = options.y || 0;
  }
}

Junction.__attrs__ = ['layer', 'x', 'y'];
