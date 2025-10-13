/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* No-Rotate Zones */
  var noRotateZones = scene.append('g').attr('class', 'no-rotate-zones');
  var zones = noRotateZones.append('g').attr('class', 'zones');

  function modelsUpdated() {
    var data = viz.models.rawNoRotateZones();
    var z = zones.selectAll('polygon').data(data).join('polygon');
    z.attrs({
      points: (d) => d.points
    });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
