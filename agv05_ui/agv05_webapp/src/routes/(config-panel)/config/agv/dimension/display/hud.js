/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import cursorInfoF from 'map-layout-editor/hud/cursor-info';

export default function (viz) {
  /**
   * Layer: Heads-up Display
   * HUDs are static with respect to screen coordinate
   */
  var hud = viz.append('g');
  hud.attrs({
    class: 'hud',
    transform: 'translate(0,30)'
  });

  cursorInfoF(viz, hud);

  var miniAxis = hud.append('g');
  miniAxis.attrs({
    class: 'mini-axis',
    transform: 'translate(18,0)'
  });

  miniAxis.append('path').attrs({
    d: 'M 20 0 H 0 V 20 M 16 -3 L 20 0 L 16 3 M -4 17 L 0 20 L 4 16'
  });
  miniAxis.append('text').text('x').attrs({
    x: 25,
    y: 3
  });
  miniAxis.append('text').text('y').attrs({
    x: 0,
    y: 28
  });

  hud.zoomed = hud.resized = hud.modelsUpdated = () => undefined;

  return hud;
}
