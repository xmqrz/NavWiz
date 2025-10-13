/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

export default function (viz) {
  /* Pan & Zoom */
  var zoom = d3
    .zoom()
    .clickDistance(3)
    .tapDistance(3)
    .scaleExtent(viz.meta.scaleExtent)
    .wheelDelta(wheelDelta)
    .on('zoom', zoomed);
  var no_zoom = d3.zoom();

  function zoomed(event) {
    viz.meta.scale = event.transform.k;
    viz.meta.translate = [event.transform.x, event.transform.y];
    viz.zoomed();
  }

  // make ctrl key not behave as fast scroll
  function wheelDelta(event) {
    return -event.deltaY * (event.deltaMode === 1 ? 0.05 : event.deltaMode ? 1 : 0.002);
  }

  // Hack: make ctrl key neccessary to trigger scroll zoom.
  function wrapMouseWheel(g, type) {
    var mousewheeled = g.on(type);
    if (mousewheeled) {
      g.on(type, function (event) {
        if (event.ctrlKey) {
          mousewheeled.apply(this, [event]);
          event.preventDefault();
        }
      });
    }
  }

  function enable(en = true) {
    if (en) {
      viz.call(zoom);
      viz.call(wrapMouseWheel, 'wheel.zoom');
      viz.call(wrapMouseWheel, 'mousewheel.zoom');
      viz.call(wrapMouseWheel, 'MozMousePixelScroll.zoom');
      viz.call(zoom.transform, getTransform());
    } else {
      viz.call(no_zoom);
    }
  }

  function zoomIn() {
    viz.call(zoom.scaleBy, 1.25);
  }

  function zoomOut() {
    viz.call(zoom.scaleBy, 0.8);
  }

  function zoomFit() {
    viz.call(zoom.transform, d3.zoomIdentity);
  }

  function zoomCenter(x, y) {
    viz.call(zoom.transform, new d3.ZoomTransform(1, x, y));
  }

  function zoomTranslate(x, y) {
    viz.call(zoom.transform, new d3.ZoomTransform(viz.meta.scale, x, y));
  }

  function resized() {}

  function getTransform() {
    return new d3.ZoomTransform(viz.meta.scale, viz.meta.translate[0], viz.meta.translate[1]);
  }

  return {
    enable: enable,
    zoomIn: zoomIn,
    zoomOut: zoomOut,
    zoomFit: zoomFit,
    zoomCenter: zoomCenter,
    zoomTranslate: zoomTranslate,
    resized: resized
  };
}
