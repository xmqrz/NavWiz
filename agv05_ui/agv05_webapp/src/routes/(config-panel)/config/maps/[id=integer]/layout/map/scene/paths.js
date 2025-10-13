/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Paths */
  var paths = scene.append('g').attr('class', 'paths');
  var Utils = scene.Utils;

  function modelsUpdated() {
    var data = viz.models.rawPaths();
    var dataJ = viz.models.rawJunctions();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();

    var p = paths.selectAll('path').data(data).join('path');
    p.attrs({
      d: function (d) {
        if (d.j1 >= 0 && d.j1 < dataJ.length && d.j2 >= 0 && d.j2 < dataJ.length) {
          return Utils.generateConnectorPath(d, dataJ[d.j1], dataJ[d.j2]);
        } else {
          console.log("Path's junction index out of bounds.");
          return '';
        }
      },
      class: function (d, idx) {
        let c = ['path', `pid-${idx}`];
        if (d.bj1 >= 0) {
          c.push(`bj1-${d.bj1}`);
        }
        if (d.bj2 >= 0) {
          c.push(`bj2-${d.bj2}`);
        }
        return c.join(' ');
      }
    }).classed('active', function (d) {
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
