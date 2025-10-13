/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

export default function (viz, options) {
  /* Options */
  var _options = Object.assign(
    {
      requireCtrlKey: false,
      // Ratio: top, right, bottom, left
      // ** top + bottom = 1.0, left + right = 1.0 **
      zoomFitEmptyRatio: [0.5, 0.5, 0.5, 0.5]
    },
    options
  );

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
    if (!Number.isNaN(event.transform.k)) {
      viz.meta.scale = event.transform.k;
    }
    if (!Number.isNaN(event.transform.x) && !Number.isNaN(event.transform.y)) {
      viz.meta.translate = [event.transform.x, event.transform.y];
    }
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

  function enable(en) {
    if (!arguments.length || en) {
      viz.call(zoom);
      if (_options.requireCtrlKey) {
        viz.call(wrapMouseWheel, 'wheel.zoom');
        viz.call(wrapMouseWheel, 'mousewheel.zoom');
        viz.call(wrapMouseWheel, 'MozMousePixelScroll.zoom');
      }
    } else {
      viz.call(no_zoom);
    }
  }

  function disablePan() {
    viz
      .on('mousedown.zoom', null)
      .on('mousemove.zoom', null)
      .on('mouseup.zoom', null)
      .on('touchstart.zoom', null);
  }

  function coordToPixel(pt) {
    return d3.zoomTransform(viz.node()).apply(pt);
  }

  function pixelToCoord(pt) {
    return d3.zoomTransform(viz.node()).invert(pt);
  }

  function getCenter() {
    return pixelToCoord([viz.meta.width / 2, viz.meta.height / 2]);
  }

  function getCorners() {
    var leftTop = pixelToCoord([0, 0]);
    var rightBottom = pixelToCoord([viz.meta.width, viz.meta.height]);
    return {
      left: leftTop[0],
      right: rightBottom[0],
      top: leftTop[1],
      bottom: rightBottom[1]
    };
  }

  function getSnapDistance() {
    if (viz.meta.scale > 25) {
      return 0.2;
    } else {
      return 1.0;
    }
  }

  function getLowerSnapDistance() {
    if (viz.meta.scale > 25) {
      return null;
    } else {
      return 0.2;
    }
  }

  function computeSnap(pt, snapDistance) {
    var sd = snapDistance || getSnapDistance();
    return [Math.round(pt[0] / sd) * sd, Math.round(pt[1] / sd) * sd];
  }

  function zoomIn() {
    viz.call(zoom.scaleBy, 1.25);
  }

  function zoomOut() {
    viz.call(zoom.scaleBy, 0.8);
  }

  function zoomFit() {
    var inset = 0.8;
    var bbox = viz.models && viz.models.computeBoundingBox();

    if (!bbox) {
      // if no elements yet.
      var ratio = _options.zoomFitEmptyRatio;
      bbox = {};
      bbox.left = (-viz.meta.width / viz.meta.scale) * ratio[3] * inset;
      bbox.right = (viz.meta.width / viz.meta.scale) * ratio[1] * inset;
      bbox.top = (-viz.meta.height / viz.meta.scale) * ratio[0] * inset;
      bbox.bottom = (viz.meta.height / viz.meta.scale) * ratio[2] * inset;
    }

    var scaleX = viz.meta.width / (bbox.right - bbox.left);
    var scaleY = viz.meta.height / (bbox.bottom - bbox.top);
    var scale = Math.min(scaleX, scaleY) * inset;

    var x = (bbox.left + bbox.right) / 2;
    var y = (bbox.top + bbox.bottom) / 2;

    x -= viz.meta.width / 2 / scale;
    y -= viz.meta.height / 2 / scale;
    x = x * -scale;
    y = y * -scale;

    viz.call(zoom.transform, new d3.ZoomTransform(scale, x, y));
  }

  function zoomCenter(x, y) {
    let scale = 70;
    viz.call(
      zoom.transform,
      new d3.ZoomTransform(
        scale,
        viz.meta.width / 2 - x * scale,
        viz.meta.height / 2 - y * scale
      )
    );
  }

  function recenter(c) {
    viz.call(
      zoom.transform,
      new d3.ZoomTransform(
        viz.meta.scale,
        viz.meta.width / 2 - c[0] * viz.meta.scale,
        viz.meta.height / 2 - c[1] * viz.meta.scale
      )
    );
  }

  function resized() {}

  return {
    enable: enable,
    disablePan: disablePan,
    coordToPixel: coordToPixel,
    pixelToCoord: pixelToCoord,
    getCenter: getCenter,
    getCorners: getCorners,
    getSnapDistance: getSnapDistance,
    getLowerSnapDistance: getLowerSnapDistance,
    computeSnap: computeSnap,
    zoomIn: zoomIn,
    zoomOut: zoomOut,
    zoomFit: zoomFit,
    zoomCenter: zoomCenter,
    recenter: recenter,
    resized: resized,
    getZoom: () => zoom
  };
}
