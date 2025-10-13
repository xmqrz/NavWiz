/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

import defF from 'mapx-layout-editor/defs';
import sceneF from './scene';
import hudF from './hud';
import zoomF from 'map-layout-editor/zoom';

const PAN_LIMIT = 2; // meters

export default function ($viz, payloadIdx) {
  /* Select svg object */
  var viz = d3.select($viz[0]);

  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    scale: 200, // pixels per 1 meter
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
  viz.defs = defF(viz);

  /* Layers: Scene & Heads-Up Display */
  viz.scene = sceneF(viz, payloadIdx);
  viz.hud = hudF(viz);

  /* Pan & Zoom */
  viz.zoom = zoomF(viz, {
    requireCtrlKey: true
  });
  viz.zoom.getZoom().translateExtent([
    [-PAN_LIMIT, -PAN_LIMIT],
    [PAN_LIMIT, PAN_LIMIT]
  ]);
  viz.zoom.enable();

  // monkey-patch
  var _getSnapDistance = viz.zoom.getSnapDistance;
  viz.zoom.getSnapDistance = function () {
    if (viz.meta.scale > 100) {
      return 0.05;
    }
    return _getSnapDistance();
  };

  /* Zoom Updates */
  viz.zoomed = function () {
    viz.scene.zoomed();
    viz.hud.zoomed();
  };

  /* Dimension Updates */
  viz.resized = function () {
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
    viz.zoom.zoomFit();
  };

  return viz;
}
