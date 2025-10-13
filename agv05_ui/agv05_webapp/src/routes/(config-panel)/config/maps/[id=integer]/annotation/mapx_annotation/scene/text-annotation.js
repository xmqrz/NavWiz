/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default function (viz, scene) {
  /* TextAnnotation */
  var textAnnotations = scene.append('g').attr('class', 'text-annotations');

  function modelsUpdated() {
    var data = viz.models.rawTextAnnotations();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();
    let processedData = [];
    for (let d of data) {
      let fontSize = d.size;
      let lines = d.content ? d.content.split('\n') : [];
      let baseTextX = d.x;
      let textY = d.y;
      let sub = [];
      for (let i = 0; i < lines.length; ++i) {
        let textX = baseTextX - i * fontSize * 1.2;
        sub.push({
          content: lines[i],
          size: `${d.size}pt`,
          x: textX,
          y: textY,
          parent: d
        });
      }
      processedData.push(sub);
    }

    var ta = textAnnotations
      .selectAll('g')
      .data(processedData)
      .join('g')
      .attrs({
        id: (d, idx) => `textAnnotation-id-${idx}`
      });

    var t = ta
      .selectAll('text')
      .data((d) => d)
      .join('text');
    t.text((d) => d.content);
    t.attrs({
      class: 'text-annotation',
      x: (d) => d.y * -1,
      y: (d) => d.x * -1,
      'font-size': (d) => d.size,
      transform: 'rotate(90) scale(-1,1)'
    }).classed('active', function (d) {
      return (
        discardObjects.indexOf(d.parent) < 0 &&
        (activeObjects.indexOf(d.parent) >= 0 || hoverObjects.indexOf(d.parent) >= 0)
      );
    });
  }

  function toggle(state) {
    textAnnotations.classed('hidden', !state);
  }

  return {
    modelsUpdated: modelsUpdated,
    toggle: toggle
  };
}
