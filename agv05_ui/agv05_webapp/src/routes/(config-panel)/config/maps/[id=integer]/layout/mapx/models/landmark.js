/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tang Swee Ho
 */

export default class Landmark {
  constructor(options) {
    options = options || {};
    this.x = options.x || 0;
    this.y = options.y || 0;
  }
}

Landmark.__attrs__ = ['x', 'y', 'rfid'];
