/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import EditMode from './edit-mode';

import canvasF from './canvas';
import utilsF from 'mapx-layout-editor/scene/utils';
import gridF from 'mapx-layout-editor/scene/grid';
import constructF from './construct';
import liveConstructF from 'mapx-layout-editor/scene/live-construct';
import liveF from 'mapx-layout-editor/scene/live';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var canvas = canvasF(viz);
  var scene = viz.svg.append('g').attr('class', 'scene');
  scene.Utils = utilsF(viz, scene);

  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var construct = constructF(viz, scene);
  var liveConstruct = liveConstructF(viz, scene, EditMode);

  scene.live = liveF(viz, scene);

  scene.zoomed = function () {
    canvas.zoomed();
    scene.attr(
      'transform',
      `translate(${viz.meta.translate})scale(${viz.meta.scale})scale(1,-1)rotate(90)`
    );
    if (grid) {
      grid.zoomed();
    }
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    canvas.modelsUpdated();
  };

  scene.mode = EditMode.POINTER;
  scene.setEditMode = function (m) {
    construct.setEditMode(m);
    liveConstruct.setEditMode(m); // Override construct for live edit mode
    scene.mode = m;
  };

  scene.showGrid = grid ? grid.show : () => void 0;
  scene.hideGrid = grid ? grid.hide : () => void 0;

  return scene;
}
