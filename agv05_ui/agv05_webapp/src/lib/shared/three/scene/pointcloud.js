/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as THREE from 'three';

export default function (scene, hasColor) {
  var mat = new THREE.PointsMaterial({
    color: hasColor ? 0xffffff : 0x000000,
    size: hasColor ? 6.0 : 4.0,
    sizeAttenuation: false,
    opacity: 1.0,
    vertexColors: !!hasColor
  });
  var geom = new THREE.BufferGeometry();
  var cloud = new THREE.Points(geom, mat);
  scene.add(cloud);

  function update(data, visible) {
    cloud.visible = visible;
    if (!visible) return;

    var buffer;
    if (!hasColor) {
      buffer = new THREE.InterleavedBuffer(new Float32Array(data), 4);
      geom.setAttribute('position', new THREE.InterleavedBufferAttribute(buffer, 3, 0));
      geom.computeBoundingSphere();
    } else {
      buffer = new THREE.InterleavedBuffer(new Float32Array(data), 8);
      geom.setAttribute('position', new THREE.InterleavedBufferAttribute(buffer, 3, 0));
      buffer = new THREE.InterleavedBuffer(new Uint8Array(data), 32);
      geom.setAttribute('color', new THREE.InterleavedBufferAttribute(buffer, 3, 16, true));
      geom.computeBoundingSphere();
    }
  }

  return { update };
}
