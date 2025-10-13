/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class ForbiddenZone {
  constructor(options) {
    options = options || {};
    this.points = options.points || [];
  }
}

ForbiddenZone.__attrs__ = ['points'];
