/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Forbidden Zones */
  var forbiddenZones = scene.append('g').attr('class', 'forbidden-zones');
  var zones = forbiddenZones.append('g').attr('class', 'zones');

  function modelsUpdated() {
    var data = viz.models.rawForbiddenZones();
    var z = zones.selectAll('polygon').data(data).join('polygon');
    z.attrs({
      points: (d) => d.points
    });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
