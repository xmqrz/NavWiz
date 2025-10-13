/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Junctions */
  var junctions = scene.append('g').attr('class', 'junctions');

  function modelsUpdated() {
    var data = viz.models.rawJunctions();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();

    junctions
      .selectAll('use')
      .data(data)
      .join('use')
      .attr('class', 'junction')
      .attr('xlink:href', '#junction')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y)
      .attr('id', (d, idx) => `jid-${idx}`)
      .classed('active', function (d) {
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
