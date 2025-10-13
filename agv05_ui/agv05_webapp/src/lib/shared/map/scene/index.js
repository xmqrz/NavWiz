/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import utilsF from 'map-layout-editor/scene/utils';
import gridF from 'map-layout-editor/scene/grid';
import noRotateZonesF from '../../mapx/scene/no-rotate-zones';
import polygonAnnotationsF from './polygon-annotation';
import stationsF from 'map-layout-editor/scene/stations';
import pathsF from 'map-layout-editor/scene/paths';
import junctionsF from 'map-layout-editor/scene/junctions';
import robotF from 'map-layout-editor/scene/robot';
import textAnnotationF from 'map-annotation-editor/scene/text-annotation';
import iconAnnotationF from 'map-annotation-editor/scene/icon-annotation';
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
  var polygonAnnotations = polygonAnnotationsF(viz, scene);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var robot = robotF(viz, scene);
  var textAnnotations = textAnnotationF(viz, scene);
  var iconAnnotations = iconAnnotationF(viz, scene);

  scene.activeObject = activeObjectF(viz, scene, true);
  scene.hoverObject = activeObjectF(viz, scene, false);
  scene.discardObject = activeObjectF(viz, scene, false);

  scene.zoomed = function () {
    scene.attr('transform', `translate(${viz.meta.translate})scale(${viz.meta.scale})`);
    if (grid) {
      grid.zoomed();
    }
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    noRotateZones.modelsUpdated();
    stations.modelsUpdated();
    paths.modelsUpdated();
    junctions.modelsUpdated();
    robot.modelsUpdated();
    textAnnotations.modelsUpdated();
    polygonAnnotations.modelsUpdated();
    iconAnnotations.modelsUpdated();
  };

  scene.updateRobotSvg = robot.updateRobotSvg;
  scene.updateRobotPose = robot.updateRobotPose;
  scene.updateRobotMotion = robot.updateRobotMotion;
  scene.toggleRobotFocus = robot.toggleRobotFocus;

  var annotation = true;
  scene.toggleAnnotation = function () {
    annotation = !annotation;
    textAnnotations.toggle(annotation);
    polygonAnnotations.toggle(annotation);
    iconAnnotations.toggle(annotation);
  };

  return scene;
}
