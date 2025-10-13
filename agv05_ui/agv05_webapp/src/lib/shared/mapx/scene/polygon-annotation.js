/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default function (viz, scene) {
  /* Polygon Annotations */
  var polygonAnnotations = scene.append('g').attr('class', 'polygon-annotations');
  var zones = polygonAnnotations.append('g').attr('class', 'zones');

  function modelsUpdated() {
    var data = viz.models.rawPolygonAnnotations();
    var z = zones.selectAll('polygon').data(data).join('polygon');
    z.attrs({
      fill: (d) => d.fill,
      'fill-opacity': (d) => d.fillOpacity,
      points: (d) => d.points
    });
  }

  function toggle(state) {
    polygonAnnotations.classed('hidden', !state);
  }

  return {
    modelsUpdated: modelsUpdated,
    toggle: toggle
  };
}
