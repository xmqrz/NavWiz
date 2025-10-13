/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

export default function (viz, scene, muted = false) {
  /* Junctions */
  var junctions = scene.append('g').attr('class', !muted ? 'junctions' : 'junctions-muted');

  function modelsUpdated() {
    var visibleLayers = viz.models.rawVisibleLayers();
    var data = viz.models.rawJunctions();
    data = _.reduce(
      data,
      function (result, d, idx) {
        if (visibleLayers[d.layer] !== muted) {
          result.push([d, idx]);
        }
        return result;
      },
      []
    );
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();

    junctions
      .selectAll('use')
      .data(data)
      .join('use')
      .attr('xlink:href', '#junction')
      .attr('x', ([d, _]) => d.x)
      .attr('y', ([d, _]) => d.y)
      .attr('id', ([_, idx]) => `jid-${idx}`)
      .attr('class', ([d, _]) => `junction layer-${d.layer}`)
      .classed('muted', muted)
      .classed('active', function ([d, _]) {
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
