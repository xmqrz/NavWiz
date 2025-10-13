/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import defsF from 'mapx-layout-editor/defs';
import hudF from 'mapx-layout-editor/hud';
import zoomF from 'mapx-layout-editor/zoom';
import sceneF from './scene';

var PAN_LIMIT = 10; // meters

export default function (_outer, options) {
  /* Create svg object */
  var viz = d3.select(document.createElementNS('http://www.w3.org/2000/svg', 'svg'));
  viz.attr('class', 'viz laser-area-viz map-viz');

  /* Options */
  viz.options = Object.assign(
    {
      laserDownSample: false
    },
    options
  );

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 100, // pixels per 1 meter
    scaleExtent: [50, 500],
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
    zoomControls: true
  });

  /* Pan & Zoom */
  viz.zoom = zoomF(viz, {
    requireCtrlKey: true
  });
  viz.zoom.getZoom().translateExtent([
    [-PAN_LIMIT, -PAN_LIMIT],
    [PAN_LIMIT, PAN_LIMIT]
  ]);
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

    // Trigger re-center of robot and laser.
    viz.zoomed();
  };

  /* Model Updates */
  viz.modelsUpdated = function () {
    viz.scene.modelsUpdated();
    viz.hud.modelsUpdated();
  };

  return viz;
}
