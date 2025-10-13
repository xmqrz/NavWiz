/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default class PolygonAnnotation {
  constructor(options) {
    options = options || {};
    this.points = options.points || [];
    this.fill = options.fill || '#00f';
    this.fillOpacity = options.fillOpacity || 0.2;
  }
}

PolygonAnnotation.__attrs__ = ['points', 'fill', 'fillOpacity'];
