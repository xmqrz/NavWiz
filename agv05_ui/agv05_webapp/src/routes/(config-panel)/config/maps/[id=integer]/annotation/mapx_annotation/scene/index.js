/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import EditMode from './edit-mode';
import mapxUtilsF from 'mapx-layout-editor/scene/utils';
import utilsF from './utils';
import rawMapF from 'mapx-layout-editor/scene/raw-map';
import landmarksF from 'mapx-layout-editor/scene/landmarks';
import gridF from 'mapx-layout-editor/scene/grid';
import forbiddenZonesF from 'mapx-layout-editor/scene/forbidden-zones';
import noRotateZonesF from 'map-layout-editor/scene/no-rotate-zones';
import polygonAnnotationsF from '../../map_annotation/scene/polygon-annotation';
import stationsF from 'mapx-layout-editor/scene/stations';
import pathsF from 'mapx-layout-editor/scene/paths';
import junctionsF from 'mapx-layout-editor/scene/junctions';
import iconAnnotationsF from '../../map_annotation/scene/icon-annotation';
import textAnnotationsF from './text-annotation';
import constructF from './construct';
import activeObjectF from './active-object';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  var mapxUtils = mapxUtilsF(viz, scene);
  scene.Utils = utilsF(viz, scene);

  // Hack to get utils dependencies for map objects.
  scene.Utils.generateConnectorPath = mapxUtils.generateConnectorPath;

  var rawMap = rawMapF(viz, scene);
  var landmarks = landmarksF(viz, scene);
  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var forbiddenZones = forbiddenZonesF(viz, scene);
  var noRotateZones = noRotateZonesF(viz, scene);
  var polygonAnnotations = polygonAnnotationsF(viz, scene);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var iconAnnotations = iconAnnotationsF(viz, scene);
  var textAnnotations = textAnnotationsF(viz, scene);
  var construct = constructF(viz, scene);

  // append to construct element
  polygonAnnotations.construct(construct.polygonAnnotation);

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
    polygonAnnotations.zoomed();
  };

  scene.resized = function () {};

  scene.modelsLoaded = function () {
    rawMap.modelsUpdated();
    landmarks.modelsUpdated();
    forbiddenZones.modelsUpdated();
    noRotateZones.modelsUpdated();
    stations.modelsUpdated();
    paths.modelsUpdated();
    junctions.modelsUpdated();
  };

  scene.modelsUpdated = function () {
    textAnnotations.modelsUpdated();
    polygonAnnotations.modelsUpdated();
    iconAnnotations.modelsUpdated();
  };

  scene.mode = EditMode.POINTER;
  scene.setEditMode = function (m) {
    construct.setEditMode(m);
    // polygonAnnotations can overwrite viz's hover event after construct
    polygonAnnotations.enableEvents(m === EditMode.POLYGON);
    scene.mode = m;
  };

  scene.destroy = function () {
    construct.destroy();
  };

  return scene;
}
