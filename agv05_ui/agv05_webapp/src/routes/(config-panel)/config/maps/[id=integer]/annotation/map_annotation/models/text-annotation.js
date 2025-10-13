/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default class TextAnnotation {
  constructor(options) {
    options = options || {};
    this.x = options.x || 0;
    this.y = options.y || 0;
    this.size = options.size || 0.4; // in pt
    this.content = options.content || '';
  }
}

TextAnnotation.__attrs__ = ['x', 'y', 'size', 'content'];
