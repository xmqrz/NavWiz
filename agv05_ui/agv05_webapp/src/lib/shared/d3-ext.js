/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'node_modules/d3';

var d3ext = {};

/*** Custom d3 behavior: Hover ***/
d3ext.hover = function (enable) {
  var event = d3.dispatch('hoverstart', 'hover', 'hoverend');
  var mouseenter = 'mouseenter.hover',
    mousemove = 'mousemove.hover',
    mouseleave = 'mouseleave.hover';

  if (!enable) {
    return function (g) {
      g.on(mouseenter, null).on(mousemove, null).on(mouseleave, null);
    };
  }
  var _g;
  var hover = function (g) {
    _g = g;
    _g.on(mouseenter, mouseentered);
  };
  var mouseentered = function (e) {
    var that = this;
    _g.on(mousemove, moved).on(mouseleave, ended);
    event.call('hoverstart', hover, e);

    function moved(ee) {
      hover.mouse = d3.pointer(ee, that);
      event.call('hover', hover, ee);
    }

    function ended(ee) {
      _g.on(mousemove, null).on(mouseleave, null);
      event.call('hoverend', hover, ee);
    }
  };
  return d3ext.rebind(hover, event, 'on');
};

/*** Custom d3 behavior: Track ***/
// Advantages over d3's own drag:
// 1. Tracks mouse events even outside of the svg element.
// 2. Remembers the start point of tracking.
// 3. Has an 'enable' parameter.
// 4. Has event.trackTarget that behave similar to mouse event.target for touch event.
d3ext.track = function (enable) {
  var event = d3.dispatch('trackstart', 'track', 'trackend');
  var mousedown = 'mousedown.track',
    mousemove = 'mousemove.track',
    mouseup = 'mouseup.track';
  var touchstart = 'touchstart.track';
  // Hint: track mouse event to window to get update even when mouse left the element.
  var w = d3.select(window);

  if (!enable) {
    return function (g) {
      g.on(mousedown, null).on(touchstart, null);
    };
  }
  var _g;
  var track = function (g) {
    _g = g;
    _g.on(mousedown, mousedowned).on(touchstart, touchstarted);
  };
  var trackTo = function (a0, p0, a1, p1) {
    track.a0 = a0;
    track.a1 = a1;
    track.p0 = p0;
    track.p1 = p1;
  };
  var mousedowned = function (e) {
    var that = this;
    var absLoc0 = d3.pointer(e, window);
    var location0 = d3.pointer(e, that);
    w.on(mousemove, moved, true).on(mouseup, ended, true);
    trackTo(absLoc0, location0, null, null);
    modifyEvent(e);
    event.call('trackstart', track, e);

    function moved(ee) {
      trackTo(absLoc0, location0, d3.pointer(ee, window), d3.pointer(ee, that));
      modifyEvent(ee);
      event.call('track', track, ee);
    }

    function ended(ee) {
      w.on(mousemove, null).on(mouseup, null);
      modifyEvent(ee);
      event.call('trackend', track, ee);
    }

    function modifyEvent(ee) {
      // touch event.target always point to start elem.
      ee.trackTarget = ee.target;
    }
  };
  var touchstarted = function (e) {
    var trackId = e.changedTouches[0].identifier;
    var touchmove = 'touchmove.track-' + trackId,
      touchend = `touchend.track-${trackId} touchcancel.track-${trackId}`;
    var that = this;
    var absLoc0 = d3.pointers(e, window)[trackId];
    var location0 = d3.pointers(e, that)[trackId];
    w.on(touchmove, moved, true).on(touchend, ended, true).on(touchstart, null);
    trackTo(absLoc0, location0, null, null);
    modifyEvent(e);
    event.call('trackstart', track, e);

    function moved(ee) {
      var location1 = d3.pointers(ee, that)[trackId];
      if (!location1) {
        return;
      }
      var absLoc1 = d3.pointers(ee, window)[trackId];
      trackTo(absLoc0, location0, absLoc1, location1);
      modifyEvent(ee);
      event.call('track', track, ee);
    }

    function ended(ee) {
      w.on(touchmove, null).on(touchend, null).on(touchstart, touchstarted);
      modifyEvent(ee);
      event.call('trackend', track, ee);
    }

    function modifyEvent(ee) {
      // touch event.target always point to start elem.
      var t = ee.changedTouches[0];
      if (!t) {
        ee.trackTarget = undefined;
        return;
      }
      ee.trackTarget = document.elementFromPoint(t.clientX, t.clientY);
    }
  };
  return d3ext.rebind(track, event, 'on');
};

// FROM: https://stackoverflow.com/questions/47844765/d3-rebind-in-d3-v4
// Copies a variable number of methods from source to target.
d3ext.rebind = function (target, source) {
  var i = 1,
    n = arguments.length,
    method;
  while (++i < n) target[(method = arguments[i])] = d3_rebind(target, source, source[method]);
  return target;
};

// Method is assumed to be a standard D3 getter-setter:
// If passed with no arguments, gets the value.
// If passed with arguments, sets the value and returns the target.
function d3_rebind(target, source, method) {
  return function () {
    var value = method.apply(source, arguments);
    return value === source ? target : value;
  };
}

export default d3ext;
