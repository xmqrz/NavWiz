/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Direction from './direction';

export default class Station {
  constructor(options) {
    options = options || {};
    this.name = options.name || '';
    this.j = options.j;
    this.direction = options.direction || 0;
  }
}

Station.__attrs__ = ['name', 'j', 'direction'];
Station.Direction = Direction;
