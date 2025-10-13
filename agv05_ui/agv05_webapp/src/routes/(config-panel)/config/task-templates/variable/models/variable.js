/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default class Variable {
  constructor(options) {
    options = options || {};
    this.name = options.name;
    this.type = options.type;
    this.default = options.default;
    this.description = options.description;
  }
}

Variable.__attrs__ = ['name', 'type', 'default', 'description'];
