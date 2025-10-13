/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import EditMode from './edit-mode';
import utilsF from './utils';
import gridF from './grid';
import noRotateZonesF from './no-rotate-zones';
import locHintZonesF from './loc-hint-zones';
import stationsF from './stations';
import pathsF from './paths';
import junctionsF from './junctions';
import constructF from './construct';
import activeObjectF from './active-object';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  scene.Utils = utilsF(viz, scene);

  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var noRotateZones = noRotateZonesF(viz, scene);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var locHintZones = locHintZonesF(viz, scene);
  var construct = constructF(viz, scene);

  // append to construct element
  noRotateZones.construct(construct.noRotateZone);
  locHintZones.construct(construct.locHintZone);

  scene.activeObject = activeObjectF(viz, scene, true);
  scene.hoverObject = activeObjectF(viz, scene, false);
  scene.discardObject = activeObjectF(viz, scene, false);

  scene.zoomed = function () {
    scene.attr('transform', `translate(${viz.meta.translate})scale(${viz.meta.scale})`);
    if (grid) {
      grid.zoomed();
    }
    noRotateZones.zoomed();
    locHintZones.zoomed();
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    noRotateZones.modelsUpdated();
    locHintZones.modelsUpdated();
    stations.modelsUpdated();
    paths.modelsUpdated();
    junctions.modelsUpdated();
  };

  scene.mode = EditMode.POINTER;
  scene.setEditMode = function (m) {
    construct.setEditMode(m);
    // noRotateZones can overwrite viz's hover event after construct
    noRotateZones.enableEvents(m === EditMode.NO_ROTATE_ZONE);
    locHintZones.enableEvents(m === EditMode.LOC_HINT_ZONE);
    scene.mode = m;
  };

  return scene;
}
