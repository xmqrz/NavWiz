/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default class IconAnnotation {
  constructor(options) {
    options = options || {};
    this.x = options.x || 0;
    this.y = options.y || 0;
    this.type = options.type || 'lift';
  }
}
IconAnnotation.__attrs__ = ['x', 'y', 'type'];
