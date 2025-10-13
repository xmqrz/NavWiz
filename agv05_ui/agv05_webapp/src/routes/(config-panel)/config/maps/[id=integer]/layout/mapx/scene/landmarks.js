/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Landmarks */
  var landmarks = scene.append('g').attr('class', 'landmarks');

  function modelsUpdated() {
    var data = viz.models.rawLandmarks();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.hoverObject.getAll();
    var lm = landmarks.selectAll('use').data(data).join('use');
    lm.attrs({
      class: 'landmark',
      'xlink:href': '#landmark',
      x: (d) => d.x,
      y: (d) => d.y,
      id: (d, idx) => `lid-${idx}`
    }).classed('active', function (d) {
      return (
        discardObjects.indexOf(d) < 0 &&
        (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
      );
    });
  }

  function enlarge(large) {
    landmarks.classed('landmarks-lg', large);
  }

  return {
    modelsUpdated: modelsUpdated,
    enlarge: enlarge
  };
}
