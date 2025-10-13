/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default function (viz, scene) {
  /* IconAnnotation */
  var iconAnnotations = scene.append('g').attr('class', 'icon-annotations');

  function modelsUpdated() {
    var data = viz.models.rawIconAnnotations();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();
    var ia = iconAnnotations.selectAll('use').data(data).join('use').attrs({
      class: 'icon-annotation'
    });
    ia.attrs({
      x: (d) => d.x,
      y: (d) => d.y,
      id: (d, idx) => `icon-annotation-id-${idx}`,
      'xlink:href': (d) => `#${d.type}`
    }).classed('active', function (d) {
      return (
        discardObjects.indexOf(d) < 0 &&
        (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
      );
    });
  }

  function toggle(state) {
    iconAnnotations.classed('hidden', !state);
  }

  return {
    modelsUpdated: modelsUpdated,
    toggle: toggle
  };
}
