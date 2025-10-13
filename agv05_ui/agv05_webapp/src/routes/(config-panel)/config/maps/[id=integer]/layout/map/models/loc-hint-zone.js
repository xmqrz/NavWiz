/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default class LocHintZone {
  constructor(options) {
    options = options || {};
    this.name = options.name || '';
    this.points = options.points || [];
  }
}

LocHintZone.__attrs__ = ['name', 'points'];
