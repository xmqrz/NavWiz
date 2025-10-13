/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import EditMode from './edit-mode';

import utilsF from './utils';
import rawMapF from './raw-map';
import landmarksF from './landmarks';
import gridF from './grid';
import forbiddenZonesF from './forbidden-zones';
import noRotateZonesF from 'map-layout-editor/scene/no-rotate-zones';
import locHintZonesF from 'map-layout-editor/scene/loc-hint-zones';
import stationsF from './stations';
import pathsF from './paths';
import junctionsF from './junctions';
import constructF from './construct';
import liveConstructF from './live-construct';
import liveF from './live';
import activeObjectF from './active-object';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  scene.Utils = utilsF(viz, scene);

  var rawMap = rawMapF(viz, scene);
  var landmarks = landmarksF(viz, scene);
  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var forbiddenZones = forbiddenZonesF(viz, scene);
  var noRotateZones = noRotateZonesF(viz, scene);
  var stationsMuted = stationsF(viz, scene, true);
  var pathsMuted = pathsF(viz, scene, true);
  var junctionsMuted = junctionsF(viz, scene, true);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var locHintZones = locHintZonesF(viz, scene, false);
  var construct = constructF(viz, scene);
  var liveConstruct = liveConstructF(viz, scene, EditMode);

  // append to construct element
  forbiddenZones.construct(construct.forbiddenZone);
  noRotateZones.construct(construct.noRotateZone);
  locHintZones.construct(construct.locHintZone);

  scene.live = liveF(viz, scene);

  scene.activeObject = activeObjectF(viz, scene, true);
  scene.hoverObject = activeObjectF(viz, scene, false);
  scene.discardObject = activeObjectF(viz, scene, false);

  scene.zoomed = function () {
    scene.attr(
      'transform',
      `translate(${viz.meta.translate})scale(${viz.meta.scale})scale(1,-1)rotate(90)`
    );
    if (grid) {
      grid.zoomed();
    }
    forbiddenZones.zoomed();
    noRotateZones.zoomed();
    locHintZones.zoomed();
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    rawMap.modelsUpdated();
    landmarks.modelsUpdated();
    forbiddenZones.modelsUpdated();
    noRotateZones.modelsUpdated();
    locHintZones.modelsUpdated();
    stationsMuted.modelsUpdated();
    pathsMuted.modelsUpdated();
    junctionsMuted.modelsUpdated();
    stations.modelsUpdated();
    paths.modelsUpdated();
    junctions.modelsUpdated();
  };

  scene.mode = EditMode.POINTER;
  scene.setEditMode = function (m) {
    landmarks.enlarge(m === EditMode.LANDMARK);
    construct.setEditMode(m);
    liveConstruct.setEditMode(m); // Override construct for live edit mode
    // forbiddenZones and noRotateZones can overwrite viz's hover event after construct
    if (m === EditMode.FORBIDDEN_ZONE) {
      locHintZones.enableEvents(false);
      noRotateZones.enableEvents(true);
      forbiddenZones.enableEvents(true); // put last to overwrite others
    } else if (m === EditMode.NO_ROTATE_ZONE) {
      locHintZones.enableEvents(false);
      forbiddenZones.enableEvents(true);
      noRotateZones.enableEvents(true); // put last to overwrite others
    } else if (m === EditMode.LOC_HINT_ZONE) {
      forbiddenZones.enableEvents(false);
      noRotateZones.enableEvents(false);
      locHintZones.enableEvents(true);
    } else {
      forbiddenZones.enableEvents(false);
      noRotateZones.enableEvents(false);
      locHintZones.enableEvents(false);
    }
    scene.mode = m;
  };

  return scene;
}
