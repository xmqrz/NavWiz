/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import defsF from './defs';
import sceneF from './scene';
import hudF from './hud';
import zoomF from './zoom';

export default function (outer, options) {
  /* Create svg object */
  var viz = d3.select(document.createElementNS('http://www.w3.org/2000/svg', 'svg'));
  viz.attr('class', 'viz map-viz agv05-map-viz');
  outer.append(viz.node());

  /* Options */
  viz.options = Object.assign(
    {
      editable: false,
      grid: false,
      changesetsUrl: null
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

  /* Layers: Scene & Heads-Up Display (HUD) */
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
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
    viz.hud.modelsUpdated();
  };

  return viz;
}
