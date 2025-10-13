/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import defsF from 'mapx-layout-editor/defs';
import sceneF from './scene';
import hudF from 'mapx-layout-editor/hud';
import zoomF from 'mapx-layout-editor/zoom';
import liveAppF from 'mapx-layout-editor/live-app';

export default function (outer, options) {
  /* Create div object containing a canvas and an svg overlay */
  var viz = d3.select(document.createElement('div'));
  viz.attr('class', 'viz ocg-viz');
  outer.append(viz.node());

  viz.canvas = viz.append('canvas').attr('class', 'viz ocg-viz');
  viz.svg = viz.append('svg').attr('class', 'viz ocg-viz map-viz');

  /* Options */
  viz.options = Object.assign(
    {
      editable: false,
      dynamic: false,
      grid: false
    },
    options
  );

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 25, // pixels per 1 meter
    scaleExtent: [10, 1000],
    translate: [0, 0]
  };
  viz.meta.scene = {
    x: -500,
    y: -500,
    width: 1000,
    height: 1000
  };
  viz.canvas.meta = viz.svg.meta = viz.meta;

  /* Import svg definitions */
  viz.svg.defs = defsF(viz.svg);

  /* Layers: Scene & Heads-Up Display (HUD) */
  viz.scene = sceneF(viz);
  viz.hud = hudF(viz.svg);

  /* Pan & Zoom */
  viz.zoom = zoomF(viz, {
    requireCtrlKey: true
  });
  viz.zoom.enable();
  viz.canvas.zoom = viz.svg.zoom = viz.zoom;

  /* Apps */
  viz.liveApp = liveAppF(viz);

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

  viz.destroy = function () {
    viz.liveApp.disable();
  };

  return viz;
}
