/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default function (viz, scene) {
  /* Particle Cloud */
  var particleCloud = scene.append('g').attr('class', 'particle-cloud');

  function updateParticleCloud(data) {
    var p = particleCloud.selectAll('use').data(data);
    p.join('use').attrs({
      'xlink:href': '#particle',
      class: 'particle'
    });
    p.attrs({
      transform: (d) =>
        `translate(${d.position.x}, ${d.position.y})rotate(${quartenionToYaw(d.orientation)})`
    });
  }

  function quartenionToYaw(q) {
    // yaw (z-axis rotation)
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return (Math.atan2(siny_cosp, cosy_cosp) / Math.PI) * 180;
  }

  return {
    updateParticleCloud: updateParticleCloud
  };
}
