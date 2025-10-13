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
      let textX = d.x;
      let baseTextY = d.y;
      let sub = [];
      for (let i = 0; i < lines.length; ++i) {
        let textY = baseTextY + i * fontSize * 1.2;
        sub.push({
          content: lines[i],
          size: d.size,
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
      .join('text')
      .attrs({
        class: 'text-annotation'
      });
    t.text((d) => d.content);
    t.attrs({
      x: (d) => d.x,
      y: (d) => d.y,
      'font-size': (d) => `${d.size}pt`
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
