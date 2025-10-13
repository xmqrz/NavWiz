/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';
import contextMenuF from '$lib/shared/context-menu.js';

let contextMenu = contextMenuF();

export default function (viz, scene) {
  /* Quality Map */
  var qmap = scene.append('g');
  qmap.attr('class', 'qmap');
  var overlay = qmap.append('rect');
  overlay.attr({
    fill: 'white',
    opacity: 0.5
  });
  var png = qmap.append('image');

  var meta;

  function updateQmap(data, url) {
    // offset by half a pixel
    var offX = data.x0 - data.resolution / 2;
    var offY = data.y0 - data.resolution / 2;

    qmap.attr('transform', `translate(${offX},${offY})scale(${data.resolution})`);

    overlay.attr('width', data.width).attr('height', data.height).attr('display', 'block');

    png.attr('xlink:href', url).attr('width', data.width).attr('height', data.height);

    meta = data;
    viz.call(hover);
    viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.
  }

  function clearQmap() {
    if (overlay) {
      overlay.attr('display', 'none');
    }
    png.attr('xlink:href', null);
    viz.call(noHover);
  }

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered(event) {
    /* jshint validthis: true */
    var pt = worldToGrid(viz.zoom.pixelToCoord(this.mouse));
    if (!pt) {
      contextMenu.hide();
      return;
    }

    var pixel = viz.models.qmapPixel(pt[0], pt[1]);
    if (!pixel.count || pixel.avg > 100 || pixel.min > 100 || pixel.max > 100) {
      contextMenu.hide();
      return;
    }

    contextMenu.setContents(
      `<li><a href="#">Avg: ${pixel.avg}%<br/>Max: ${pixel.max}%<br/>Min: ${pixel.min}%<br/>Observations: ${pixel.count}</a></li>`
    );
    contextMenu.show(event.pageX, event.pageY);
  }

  function hoverended() {
    contextMenu.hide();
  }

  function worldToGrid(pt) {
    var mx = Math.floor((pt[0] - meta.x0) / meta.resolution + 0.5);
    var my = Math.floor((pt[1] - meta.y0) / meta.resolution + 0.5);
    if (mx >= 0 && mx < meta.width && my >= 0 && my < meta.height) {
      return [mx, my];
    }
  }

  return {
    updateQmap: updateQmap,
    clearQmap: clearQmap
  };
}
