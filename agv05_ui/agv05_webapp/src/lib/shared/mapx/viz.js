/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';
import defsF from 'mapx-layout-editor/defs';
import sceneF from './scene';
import modelsF from './models';
import hudF from 'mapx-layout-editor/hud';
import zoomF from 'mapx-layout-editor/zoom';

export default function (svg, options) {
  /* Select svg object */
  var viz = d3.select(svg);
  viz.attr('class', 'viz map-viz agv05x-map-viz');

  /* Options */
  viz.options = Object.assign(
    {
      grid: false,
      mapLayout: false,
      laserDownSample: false
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

  /* Import svg definitions */
  viz.defs = defsF(viz);

  /* Layers: Scene & Heads-Up Display (HUD) */
  viz.scene = sceneF(viz);
  viz.hud = hudF(viz, {
    zoomControls: true,
    agvControls: true,
    annotationControls: true
  });

  /* Pan & Zoom */
  viz.zoom = zoomF(viz, {
    zoomFitEmptyRatio: [1.0, 0.5, 0, 0.5]
  });
  viz.zoom.enable();

  /* Zoom Updates */
  viz.zoomed = function () {
    viz.scene.zoomed();
    viz.hud.zoomed();
  };

  /* Dimension Updates */
  viz.resized = function (width, height) {
    viz.meta.width = width;
    viz.meta.height = height;

    viz.zoom.resized();
    viz.scene.resized();
    viz.hud.resized();
  };

  /* Model Updates */
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
    viz.hud.modelsUpdated();
  };

  if (viz.options.mapLayout) {
    modelsF(viz);
  }

  return viz;
}
