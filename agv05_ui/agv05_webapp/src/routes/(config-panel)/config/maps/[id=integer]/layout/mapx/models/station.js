/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Heading from './heading';

export default class Station {
  constructor(options) {
    options = options || {};
    this.layer = options.layer || 0;
    this.name = options.name || '';
    this.j = options.j;
    this.heading = options.heading || 0;
  }
}

Station.__attrs__ = ['layer', 'name', 'j', 'heading'];
Station.Heading = Heading;
