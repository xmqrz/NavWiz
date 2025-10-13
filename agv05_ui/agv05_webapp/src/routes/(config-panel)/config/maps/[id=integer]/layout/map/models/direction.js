/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

var NORTH = 0;
var EAST = 1;
var SOUTH = 2;
var WEST = 3;
var NA = 4;

function isNS(direction) {
  return [NORTH, SOUTH].includes(direction);
}

function isEW(direction) {
  return [EAST, WEST].includes(direction);
}

export default {
  /* Direction enum */
  NORTH: NORTH,
  EAST: EAST,
  SOUTH: SOUTH,
  WEST: WEST,
  NA: NA,
  isNS: isNS,
  isEW: isEW
};
