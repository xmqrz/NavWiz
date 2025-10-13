/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class Param {
  constructor(options) {
    options = options || {};
    this.name = options.name;
    this.type = options.type;
    this.description = options.description || '';
    this.default = options.default;
    this.min = options.min;
    this.max = options.max;
    this.version = options.version;
    this.key = this.name;
    if (this.version) {
      this.key += ':' + this.version;
    }
  }
}

Param.__metaAttrs__ = ['name', 'type', 'description', 'default', 'min', 'max'];
Param.__attrs__ = ['name', 'type', 'description', 'default'];
