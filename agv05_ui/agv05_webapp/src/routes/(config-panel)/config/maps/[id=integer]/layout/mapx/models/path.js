/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default class Path {
  constructor(options) {
    options = options || {};
    this.layer = options.layer || 0;
    this.j1 = options.j1;
    this.j2 = options.j2;
    this.bj1 = options.bj1;
    this.bj2 = options.bj2;
    this.cp1 = options.cp1 || null;
    this.cp2 = options.cp2 || null;
    this.flow = options.flow || 0;
    this.facing = options.facing || 0;
    this.shape = options.shape || 0;
    this.dynamic = options.dynamic || false;
    this.tracked = options.tracked || false;
    this.distance = options.distance || 0;
    this.speed = options.speed || '${Unlimited}';
  }
}

Path.__attrs__ = [
  'layer',
  'j1',
  'j2',
  'bj1',
  'bj2',
  'cp1',
  'cp2',
  'flow',
  'facing',
  'shape',
  'dynamic',
  'tracked',
  'distance',
  'speed'
];
Path.Flow = {
  BI_DIRECTIONAL: 0,
  UNI_DIRECTIONAL: 1
};
Path.Facing = {
  FREE: 0,
  FORWARD_UNI: 1,
  REVERSE_UNI: 2,
  FORWARD_BI: 3,
  REVERSE_BI: 4
};
Path.Shape = {
  STRAIGHT: 0,
  BEZIER: 1
};
