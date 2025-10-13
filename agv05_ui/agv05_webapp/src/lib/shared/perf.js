/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

var session = {};

function begin(name) {
  session[name] = performance.now();
}

function end(name) {
  if (name in session) {
    var duration = (performance.now() - session[name]) / 1000;
    delete session[name];
    console.log(`$perf: Session '${name}' took ${duration} seconds.`);
  } else {
    console.log(`$perf: Ending session '${name}' which has not been started.`);
  }
}

export default {
  /* perf */
  begin: begin,
  end: end
};
