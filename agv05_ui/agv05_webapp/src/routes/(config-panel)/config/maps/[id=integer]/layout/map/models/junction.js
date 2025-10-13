/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class Junction {
  constructor(options) {
    options = options || {};
    this.x = options.x || 0;
    this.y = options.y || 0;
    this.rfid = options.rfid || '';
  }
}

Junction.__attrs__ = ['x', 'y', 'rfid'];
