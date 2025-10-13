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
    return d3.zoomTransform(viz.node()).apply([-pt[1], -pt[0]]);
  }

  function pixelToCoord(pt) {
    let coord = d3.zoomTransform(viz.node()).invert(pt);
    return [-coord[1], -coord[0]];
  }

  function getCenter() {
    return pixelToCoord([viz.meta.width / 2, viz.meta.height / 2]);
  }

  function getCorners() {
    var topLeft = pixelToCoord([0, 0]);
    var bottomRight = pixelToCoord([viz.meta.width, viz.meta.height]);
    return {
      left: topLeft[1],
      right: bottomRight[1],
      top: topLeft[0],
      bottom: bottomRight[0]
    };
  }

  function getSnapDistance() {
    if (viz.meta.scale > 500) {
      return 0.01;
    } else if (viz.meta.scale > 100) {
      return 0.05;
    } else if (viz.meta.scale > 25) {
      return 0.2;
    } else {
      return 1.0;
    }
  }

  function getLowerSnapDistance() {
    if (viz.meta.scale > 500) {
      return null;
    } else if (viz.meta.scale > 100) {
      return 0.01;
    } else if (viz.meta.scale > 25) {
      return 0.05;
    } else {
      return 0.2;
    }
  }

  function computeSnap(pt, snapDistance) {
    var sd = snapDistance || getSnapDistance();
    if (sd === 0.01) {
      // force consistent rounding errors
      return [Math.round(pt[0] / 0.01) * 0.2 * 0.05, Math.round(pt[1] / 0.01) * 0.2 * 0.05];
    }
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
      let ratio = _options.zoomFitEmptyRatio;
      bbox = {};
      bbox.left = (viz.meta.width / viz.meta.scale) * ratio[3] * inset;
      bbox.right = (-viz.meta.width / viz.meta.scale) * ratio[1] * inset;
      bbox.top = (viz.meta.height / viz.meta.scale) * ratio[0] * inset;
      bbox.bottom = (-viz.meta.height / viz.meta.scale) * ratio[2] * inset;
    }

    if (bbox.left === bbox.right) {
      // if single elements
      let ratio = _options.zoomFitEmptyRatio;
      bbox.left += (viz.meta.width / viz.meta.scale) * ratio[3] * inset;
      bbox.rigth += (-viz.meta.width / viz.meta.scale) * ratio[1] * inset;
    }
    if (bbox.top === bbox.bottom) {
      // if single elements
      let ratio = _options.zoomFitEmptyRatio;
      bbox.top += (viz.meta.height / viz.meta.scale) * ratio[0] * inset;
      bbox.bottom += (-viz.meta.height / viz.meta.scale) * ratio[2] * inset;
    }

    var scaleX = viz.meta.width / (bbox.left - bbox.right);
    var scaleY = viz.meta.height / (bbox.top - bbox.bottom);
    var scale = Math.min(scaleX, scaleY) * inset;

    var x = (bbox.top + bbox.bottom) / 2;
    var y = (bbox.left + bbox.right) / 2;

    x *= scale;
    y *= scale;
    x += viz.meta.height / 2;
    y += viz.meta.width / 2;

    viz.call(zoom.transform, new d3.ZoomTransform(scale, y, x));
  }

  function zoomCenter(x, y) {
    let scale = 70;
    x *= scale;
    y *= scale;
    x += viz.meta.height / 2;
    y += viz.meta.width / 2;
    viz.call(zoom.transform, new d3.ZoomTransform(scale, y, x));
  }

  function recenter(c) {
    var scale = viz.meta.scale;
    var x = viz.meta.height / 2 + c[0] * scale;
    var y = viz.meta.width / 2 + c[1] * scale;
    viz.call(zoom.transform, new d3.ZoomTransform(scale, y, x));
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
