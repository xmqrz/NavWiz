/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

'use strict';

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import Scene from './scene/index.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export default function (canvas, options) {
  var renderer = new THREE.WebGLRenderer({
    canvas: canvas,
    antialias: true,
    alpha: true
  });
  renderer.setClearColor(0xffffff, 1.0);
  renderer.sortObjects = false;
  renderer.setSize(1280, 720);
  renderer.shadowMap.enabled = false;

  var scene = Scene();

  var camera = new THREE.PerspectiveCamera(75, 1.778, 0.01, 1000);
  camera.position.set(1, 0, 1);

  var controls = new OrbitControls(camera, canvas);
  controls.maxPolarAngle = (120 * Math.PI) / 180;
  controls.minDistance = 1;
  controls.maxDistance = 100;
  controls.target.set(-2, 0, 0);

  var loop;
  start();

  function start() {
    loop = true;
    draw();
  }

  function stop() {
    if (loop) {
      cancelAnimationFrame(loop);
    }
    loop = null;
  }

  function draw() {
    if (!loop) {
      return;
    }
    controls.update();
    renderer.render(scene, camera);
    loop = requestAnimationFrame(draw);
  }

  function resize(width, height) {
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height);
  }

  function dispose() {
    renderer.dispose();
  }

  return {
    start: start,
    stop: stop,
    resize: resize,
    update: scene.update,
    update2: scene.update2,
    update3: scene.update3,
    updateCloudFlatten: scene.updateCloudFlatten,
    updateCloudColored: scene.updateCloudColored,
    updateColorMode: scene.updateColorMode,
    updateDetecting: scene.updateDetecting,
    dispose: dispose
  };
}
