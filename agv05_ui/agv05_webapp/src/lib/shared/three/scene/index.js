/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

'use strict';

import * as THREE from 'three';
import Axes from './axes';
import DepthCloud from './depthcloud';
import PointCloud from './pointcloud';
import Ground from './ground';

export default function (img) {
  var scene = new THREE.Scene();
  scene.background = new THREE.Color(0xefd1b5);

  var ground = Ground(scene);
  var axes = Axes(scene);
  var depthCloud = DepthCloud(scene);
  var cloudFlatten = PointCloud(scene, false);
  var cloudColored = PointCloud(scene, true);

  function updateColorMode(mode) {
    ground.updateColorMode(mode);
    depthCloud.updateColorMode(mode);
  }

  scene.update = depthCloud.update;
  scene.update2 = depthCloud.update2;
  scene.update3 = depthCloud.update3;
  scene.updateCloudFlatten = cloudFlatten.update;
  scene.updateCloudColored = cloudColored.update;
  scene.updateColorMode = updateColorMode;
  scene.updateDetecting = depthCloud.updateDetecting;
  return scene;
}
