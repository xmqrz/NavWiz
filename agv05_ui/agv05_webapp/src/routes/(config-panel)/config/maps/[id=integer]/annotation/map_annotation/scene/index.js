/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import EditMode from './edit-mode';
import mapUtilsF from 'map-layout-editor/scene/utils';
import utilsF from './utils';
import gridF from 'map-layout-editor/scene/grid';
import noRotateZonesF from 'map-layout-editor/scene/no-rotate-zones';
import polygonAnnotationsF from './polygon-annotation';
import iconAnnotationsF from './icon-annotation';
import textAnnotationsF from './text-annotation';
import stationsF from 'map-layout-editor/scene/stations';
import pathsF from 'map-layout-editor/scene/paths';
import junctionsF from 'map-layout-editor/scene/junctions';
import constructF from './construct';
import activeObjectF from './active-object';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  var mapUtils = mapUtilsF(viz, scene);
  scene.Utils = utilsF(viz, scene);

  // Hack to get utils dependencies for map objects.
  scene.Utils.generateConnectorPath = mapUtils.generateConnectorPath;

  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var noRotateZones = noRotateZonesF(viz, scene);
  var polygonAnnotations = polygonAnnotationsF(viz, scene);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var iconAnnotations = iconAnnotationsF(viz, scene);
  var textAnnotations = textAnnotationsF(viz, scene);
  var construct = constructF(viz, scene);

  polygonAnnotations.construct(construct.polygonAnnotation); // append to construct element

  scene.activeObject = activeObjectF(viz, scene, true);
  scene.hoverObject = activeObjectF(viz, scene, false);
  scene.discardObject = activeObjectF(viz, scene, false);

  scene.zoomed = function () {
    scene.attr('transform', `translate(${viz.meta.translate})scale(${viz.meta.scale})`);
    if (grid) {
      grid.zoomed();
    }
    polygonAnnotations.zoomed();
  };

  scene.resized = function () {};

  scene.modelsLoaded = function () {
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
