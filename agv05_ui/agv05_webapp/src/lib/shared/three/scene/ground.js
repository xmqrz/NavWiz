/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as THREE from 'three';

export default function (scene) {
  var mat = new THREE.MeshBasicMaterial({
    color: 0x0f7173,
    opacity: 0.6,
    transparent: true,
    side: THREE.DoubleSide
  });
  var geom = new THREE.PlaneGeometry(10, 10);
  var ground = new THREE.Mesh(geom, mat);
  scene.add(ground);

  var gridHelper = new THREE.GridHelper(10, 10, 0x888888);
  gridHelper.rotation.x = Math.PI / 2;
  scene.add(gridHelper);

  function updateColorMode(mode) {
    if (mode === 4) {
      mat.opacity = 0.0;
    } else {
      mat.opacity = 0.6;
    }
  }

  return {
    updateColorMode
  };
}
