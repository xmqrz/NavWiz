/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import defsF from 'mapx-layout-editor/defs';
import sceneF from './scene';
import hudF from 'mapx-layout-editor/hud';
import zoomF from 'mapx-layout-editor/zoom';

export default function (outer, options) {
  /* Create svg object */
  var viz = d3.select(document.createElementNS('http://www.w3.org/2000/svg', 'svg'));
  viz.attr('class', 'viz map-viz agv05x-map-viz');
  outer.append(viz.node());

  /* Options */
  viz.options = Object.assign(
    {
      editable: false,
      grid: false
    },
    options
  );

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 25, // pixels per 1 meter
    scaleExtent: [10, 100],
    translate: [0, 0]
  };
  viz.meta.scene = {
    x: -500,
    y: -500,
    width: 1000,
    height: 1000
  };

  /* Import svg definitions */
  viz.defs = defsF(viz);

  /* Layers: Map layout scene & Map annotation scene */
  viz.scene = sceneF(viz);
  viz.hud = hudF(viz);

  /* Pan & Zoom */
  viz.zoom = zoomF(viz, {
    requireCtrlKey: true
  });
  viz.zoom.enable();

  /* Zoom Updates */
  viz.zoomed = function () {
    viz.scene.zoomed();
    viz.hud.zoomed();
  };

  /* Dimension Updates */
  viz.resized = function () {
    var $viz = $(viz.node());
    viz.meta.width = $viz.width();
    viz.meta.height = $viz.height();

    viz.zoom.resized();
    viz.scene.resized();
    viz.hud.resized();
  };

  /* Model Updates */
  viz.modelsLoaded = function () {
    viz.scene.modelsLoaded();
    viz.modelsUpdated();
  };
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
    viz.hud.modelsUpdated();
  };

  viz.destroy = function () {
    viz.scene.destroy();
  };

  return viz;
}
