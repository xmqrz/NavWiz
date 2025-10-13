/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import utilsF from 'mapx-layout-editor/scene/utils';
import rawMapF from 'mapx-layout-editor/scene/raw-map';
import landmarksF from 'mapx-layout-editor/scene/landmarks';
import gridF from 'mapx-layout-editor/scene/grid';
import forbiddenZonesF from './forbidden-zones';
import noRotateZonesF from './no-rotate-zones';
import polygonAnnotationsF from './polygon-annotation';
import stationsF from 'mapx-layout-editor/scene/stations';
import pathsF from 'mapx-layout-editor/scene/paths';
import junctionsF from 'mapx-layout-editor/scene/junctions';
import targetF from './target';
import robotF from 'mapx-layout-editor/scene/robot';
import textAnnotationF from 'mapx-annotation-editor/scene/text-annotation';
import iconAnnotationF from 'map-annotation-editor/scene/icon-annotation';
import activeObjectF from './active-object';
import particleCloudF from './particle-cloud';

export default function (viz) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  scene.Utils = utilsF(viz, scene);

  var rawMap = rawMapF(viz, scene);
  var costmap = rawMapF(viz, scene);
  var landmarks = landmarksF(viz, scene);
  var grid = viz.options.grid ? gridF(viz, scene) : null;
  var forbiddenZones = forbiddenZonesF(viz, scene);
  var noRotateZones = noRotateZonesF(viz, scene);
  var polygonAnnotations = polygonAnnotationsF(viz, scene);
  var stations = stationsF(viz, scene);
  var paths = pathsF(viz, scene);
  var junctions = junctionsF(viz, scene);
  var target = targetF(viz, scene);
  var robot = robotF(viz, scene);
  var textAnnotations = textAnnotationF(viz, scene);
  var iconAnnotations = iconAnnotationF(viz, scene);

  scene.activeObject = activeObjectF(viz, scene, true);
  scene.hoverObject = activeObjectF(viz, scene, false);
  scene.discardObject = activeObjectF(viz, scene, false);

  var particleCloud = particleCloudF(viz, scene);

  costmap.toggle();

  scene.zoomed = function () {
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
    landmarks.modelsUpdated();
    forbiddenZones.modelsUpdated();
    noRotateZones.modelsUpdated();
    stations.modelsUpdated();
    paths.modelsUpdated();
    junctions.modelsUpdated();
    target.modelsUpdated();
    robot.modelsUpdated();
    textAnnotations.modelsUpdated();
    polygonAnnotations.modelsUpdated();
    iconAnnotations.modelsUpdated();
  };

  scene.updateLaserPose = robot.updateLaserPose;
  scene.updateLaserScan = robot.updateLaserScan;
  scene.updateReflectors = robot.updateReflectors;
  scene.updateDockingReflectors = robot.updateDockingReflectors;
  scene.updateDockingCloud = robot.updateDockingCloud;
  scene.updateMarkerPose = target.updateMarkerPose;
  scene.updateTargetPose = target.updateTargetPose;
  scene.updateDockingPlan = target.updateDockingPlan;
  scene.updateRobotSvg = robot.updateRobotSvg;
  scene.updateRobotPose = robot.updateRobotPose;
  scene.updateRobotPath = robot.updateRobotPath;
  scene.updateRobotMotion = robot.updateRobotMotion;
  scene.toggleRobotFocus = robot.toggleRobotFocus;
  scene.updateRawMap = rawMap.updateRawMap;
  scene.clearRawMap = rawMap.clearRawMap;
  scene.updateCostmap = costmap.updateRawMap;
  scene.clearCostmap = costmap.clearRawMap;
  scene.toggleCostmap = costmap.toggle;
  scene.updateParticleCloud = particleCloud.updateParticleCloud;

  var annotation = true;
  scene.toggleAnnotation = function () {
    annotation = !annotation;
    textAnnotations.toggle(annotation);
    polygonAnnotations.toggle(annotation);
    iconAnnotations.toggle(annotation);
  };

  return scene;
}
