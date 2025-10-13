/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default Pose2D;

function Pose2D(options) {
  /* Pose2D class */
  if (!(this instanceof Pose2D)) {
    return new Pose2D(options);
  }
  options = options || {};
  this.x = options.x || 0;
  this.y = options.y || 0;
  this.theta = options.theta || 0;
}

Pose2D.prototype.add = function (b) {
  return new Pose2D(coordAdd(this, b));
};
Pose2D.prototype.sub = function (b) {
  return new Pose2D(coordSub(this, b));
};
Pose2D.prototype.sub2 = function (b) {
  return new Pose2D(coordSub2(this, b));
};

// transform from local to global coords (a + b)
function coordAdd(a, b) {
  var cosB = Math.cos(b.theta);
  var sinB = Math.sin(b.theta);
  return {
    x: b.x + a.x * cosB - a.y * sinB,
    y: b.y + a.x * sinB + a.y * cosB,
    theta: normalizeAngle(b.theta + a.theta)
  };
}

// transform from global to local coords (a - b)
function coordSub(a, b) {
  var cosB = Math.cos(b.theta);
  var sinB = Math.sin(b.theta);
  return {
    x: (a.x - b.x) * cosB + (a.y - b.y) * sinB,
    y: -(a.x - b.x) * sinB + (a.y - b.y) * cosB,
    theta: normalizeAngle(a.theta - b.theta)
  };
}

// transform from child to parent coords (a - b)
function coordSub2(a, b) {
  var theta = a.theta - b.theta;
  var cosC = Math.cos(theta);
  var sinC = Math.sin(theta);
  return {
    x: a.x - b.x * cosC + b.y * sinC,
    y: a.y - b.x * sinC - b.y * cosC,
    theta: Math.atan2(sinC, cosC)
  };
}

function normalizeAngle(theta) {
  return Math.atan2(Math.sin(theta), Math.cos(theta));
}
