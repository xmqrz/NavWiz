/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as THREE from 'three';

export default function (scene) {
  var axes = new THREE.AxesHelper(0.5);
  scene.add(axes);
}
