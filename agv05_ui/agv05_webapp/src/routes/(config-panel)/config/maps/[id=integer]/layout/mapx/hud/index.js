/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import cursorInfoF from './cursor-info';
import miniAxisF from './mini-axis';
import zoomControlsF from './zoom-controls';
import agvControlsF from './agv-controls';
import annotationControlsF from './annotation-controls';

export default function (viz, options) {
  /**
   * Layer: Heads-up Display
   * HUDs are static with respect to screen coordinate
   */
  var hud = viz.append('g').attr('class', 'hud');

  /* Options */
  hud.options = Object.assign(
    {
      zoomControls: false,
      agvControls: false,
      annotationControls: false
    },
    options
  );

  cursorInfoF(viz, hud);
  miniAxisF(viz, hud);
  hud.options.zoomControls ? zoomControlsF(viz, hud) : null;
  hud.options.agvControls ? agvControlsF(viz, hud) : null;
  hud.options.annotationControls ? annotationControlsF(viz, hud) : null;

  hud.zoomed = function () {};

  hud.resized = function () {
    hud.attr('transform', `translate(0,${viz.meta.height})`);
  };

  hud.modelsUpdated = function () {};

  return hud;
}
