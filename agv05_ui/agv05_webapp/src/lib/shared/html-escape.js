/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

/* Reference: http://stackoverflow.com/a/12034334 */

var entityMap = {
  '&': '&amp;',
  '<': '&lt;',
  '>': '&gt;',
  '"': '&quot;',
  "'": '&#x27;',
  '/': '&#x2F;',
  '`': '&#x60;',
  ':': '&#x3A;',
  '=': '&#x3D;',
  '\t': '&#x9;'
};

export default function htmlEscape(string) {
  return String(string).replace(/[&<>"'`:=/\t]/g, function (s) {
    return entityMap[s];
  });
}
