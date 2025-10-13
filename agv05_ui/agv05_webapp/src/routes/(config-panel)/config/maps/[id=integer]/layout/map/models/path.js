/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Direction from './direction';

export default class Path {
  constructor(options) {
    options = options || {};
    this.j1 = options.j1;
    this.j2 = options.j2;
    this.bj1 = options.bj1;
    this.bj2 = options.bj2;
    this.direction = options.direction || 0;
    this.flow = options.flow || 0;
    this.facing = options.facing || 0;
    this.shape = options.shape || 0;
    this.distance = options.distance || 0;
    this.speed = options.speed || '${Unlimited}';
  }

  static isNS(path) {
    return Direction.isNS(path.direction);
  }

  static isEW(path) {
    return Direction.isEW(path.direction);
  }

  static getReverseDirection(path) {
    let transform = [0, 0, 3, 1, 0][path.shape];
    return (path.direction + transform + 2) % 4;
  }
}

Path.__attrs__ = [
  'j1',
  'j2',
  'bj1',
  'bj2',
  'direction',
  'flow',
  'facing',
  'shape',
  'distance',
  'speed'
];

Path.Direction = Direction;
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
  S_CURVE: 1,
  BEND_LEFT: 2,
  BEND_RIGHT: 3
};
