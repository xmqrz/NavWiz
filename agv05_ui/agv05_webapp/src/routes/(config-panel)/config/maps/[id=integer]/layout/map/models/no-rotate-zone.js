/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class NoRotateZone {
  constructor(options) {
    options = options || {};
    this.points = options.points || [];
  }
}

NoRotateZone.__attrs__ = ['points'];
