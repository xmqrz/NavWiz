/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';
import sceneF from './scene';
import zoomF from './zoom';

export default function (options) {
  /* Create svg object */
  var viz = d3.select(document.createElementNS('http://www.w3.org/2000/svg', 'svg'));
  viz.attr('class', 'viz agv05-task-template');

  /* Options */
  viz.options = Object.assign(
    {
      editable: false
    },
    options
  );

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 1,
    scaleExtent: [0.1, 2],
    translate: [0, 0]
  };
  viz.meta.scene = {
    x: -500,
    y: -500,
    width: 1000,
    height: 1000
  };

  viz.meta.manualScene = {
    minY: 0,
    maxY: 0,
    minX: 0
  };

  /* Import svg definitions */
  //viz.defs = import('./defs')(viz);

  /* Layers: Scene */
  viz.scene = sceneF(viz);

  // /* Pan & Zoom */
  viz.zoom = zoomF(viz);

  /* Zoom Updates */
  viz.zoomed = function () {
    viz.scene.zoomed();
  };

  /* Dimension Updates */
  viz.resized = function () {
    var $viz = $(viz.node());
    viz.meta.width = $viz.width();
    viz.meta.height = $viz.height();

    viz.zoom.resized();
    viz.scene.resized();
  };

  /* Model Updates */
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
  };

  /* Misc */
  viz.init = function () {
    viz.zoom.enable();
  };

  return viz;
}
