/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz) {
  var canvas = viz.canvas;
  var ctx = canvas.node().getContext('2d', { willReadFrequently: true });

  /* Metadata for canvas */
  var meta = {
    width: 0,
    height: 0,
    scale: 1,
    translate: [0, 0]
  };

  canvas.zoomed = function () {
    var metadata = viz.models.metadata();
    var resolution = metadata.resolution;
    meta.translate = viz.zoom.coordToPixel([
      // offset by half a pixel
      metadata.x0 - resolution / 2,
      metadata.y0 - resolution / 2
    ]);
    meta.scale = viz.meta.scale * resolution;
    var transform = `translate(${meta.translate[0]}px,${meta.translate[1]}px)scale(${meta.scale})scale(1,-1)rotate(90deg)`;
    canvas.style('transform', transform);
  };

  canvas.modelsUpdated = function () {
    var metadata = viz.models.metadata();
    meta.width = metadata.width;
    meta.height = metadata.height;

    canvas
      .property('width', metadata.width)
      .property('height', metadata.height)
      .styles({
        width: `${metadata.width}px`,
        height: `${metadata.height}px`
      })
      .zoomed();

    ctx.clearRect(0, 0, metadata.width, metadata.height);
    try {
      ctx.drawImage(
        metadata.png || viz.models.png(),
        0,
        0,
        metadata.width - metadata.x_shift,
        metadata.height - metadata.y_shift,
        metadata.x_shift,
        metadata.y_shift,
        metadata.width - metadata.x_shift,
        metadata.height - metadata.y_shift
      );
    } catch (e) {
      // source image is empty.
    }
  };

  canvas.renderPng = function () {
    var rc = canvas.rc.apply(null, arguments);
    ctx.clearRect(rc.x, rc.y, rc.w, rc.h);
    try {
      ctx.drawImage(viz.models.png(), rc.x, rc.y, rc.w, rc.h, rc.x, rc.y, rc.w, rc.h);
    } catch (e) {
      // source image is empty.
    }
  };

  canvas.drawLine = function (x0, y0, x1, y1) {
    // Draw line without the anti-aliasing effect.
    // Reference: http://rosettacode.org/wiki/Bitmap/Bresenham's_line_algorithm#JavaScript
    var rc = canvas.rc.apply(null, arguments);
    var dx = rc.w - 1;
    var dy = rc.h - 1;
    var sx = x0 < x1 ? 1 : -1;
    var sy = y0 < y1 ? 1 : -1;
    var err = (dx > dy ? dx : -dy) / 2;

    // Get pixel data
    var imageData = ctx.getImageData(rc.x, rc.y, rc.w, rc.h);
    var data = new Uint32Array(imageData.data.buffer);
    var idx = x0 - rc.x + (y0 - rc.y) * rc.w;
    var ix = sx;
    var iy = sy * rc.w;

    while (true) {
      data[idx] = 0xff000000; // black (in format AABBGGRR)
      if (x0 === x1 && y0 === y1) {
        break;
      }
      var e2 = err;
      if (e2 > -dx) {
        err -= dy;
        x0 += sx;
        idx += ix;
      }
      if (e2 < dy) {
        err += dx;
        y0 += sy;
        idx += iy;
      }
    }
    ctx.putImageData(imageData, rc.x, rc.y);
  };

  canvas.drawRectangle = function () {
    var rc = canvas.rc.apply(null, arguments);
    if (rc.w <= 1 || rc.h <= 1) {
      return canvas.drawLine.apply(null, arguments);
    }
    ctx.strokeStyle = 'black';
    ctx.strokeRect(rc.x + 0.5, rc.y + 0.5, rc.w - 1, rc.h - 1);
  };

  canvas.drawWhiteRectangle = function () {
    var rc = canvas.rc.apply(null, arguments);
    ctx.fillStyle = 'white';
    ctx.fillRect(rc.x, rc.y, rc.w, rc.h);
  };

  canvas.drawWhiteRectangle2 = function () {
    var rc = canvas.rc.apply(null, arguments);

    // Get pixel data
    var imageData = ctx.getImageData(rc.x, rc.y, rc.w, rc.h);
    var data = new Uint32Array(imageData.data.buffer);
    var idx = 0;
    var i, j;
    for (i = 0; i < imageData.height; ++i) {
      for (j = 0; j < imageData.width; ++j) {
        if (data[idx] !== 0xff000000) {
          data[idx] = 0xffffffff; // white (in format AABBGGRR)
        }
        ++idx;
      }
    }
    ctx.putImageData(imageData, rc.x, rc.y);
  };

  canvas.clearRectangle = function () {
    var rc = canvas.rc.apply(null, arguments);
    ctx.clearRect(rc.x, rc.y, rc.w, rc.h);
  };

  canvas.rc = function (x0, y0, x1, y1) {
    return {
      x: Math.min(x0, x1),
      y: Math.min(y0, y1),
      w: Math.abs(x1 - x0) + 1,
      h: Math.abs(y1 - y0) + 1
    };
  };

  canvas.ptAtDiv = function (pt) {
    var x = meta.translate[1] - pt[1] * meta.scale;
    var y = meta.translate[0] - pt[0] * meta.scale;
    return [x, y];
  };

  canvas.ptFromDiv = function (pt) {
    var x = Math.floor((meta.translate[1] - pt[1]) / meta.scale);
    var y = Math.floor((meta.translate[0] - pt[0]) / meta.scale);
    return [x, y];
  };

  canvas.withinCorners = function (pt) {
    return pt[0] >= 0 && pt[0] < meta.width && pt[1] >= 0 && pt[1] < meta.height;
  };

  canvas.limitMovement = function (initial_pt, final_pt) {
    var diff0 = final_pt[0] - initial_pt[0];
    var diff1 = final_pt[1] - initial_pt[1];
    if (Math.abs(diff0) < Math.abs(diff1)) {
      return [initial_pt[0], final_pt[1]];
    } else {
      return [final_pt[0], initial_pt[1]];
    }
  };

  return canvas;
}
