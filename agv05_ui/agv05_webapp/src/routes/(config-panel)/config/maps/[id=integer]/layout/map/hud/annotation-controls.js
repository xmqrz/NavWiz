/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default function (viz, hud) {
  /* Annotation Controls */
  var annotationControls = hud.append('g').attr('class', 'annotation-controls');
  var annotationToggle = annotationControls
    .append('use')
    .attr('xlink:href', '#annotation-toggle')
    .attr('tip-title', 'Toggle annotation display')
    .attr('x', 26)
    .attr('y', -150);

  var annotation = true;

  annotationToggle.on('click', function (event) {
    event.stopPropagation();
    viz.scene.toggleAnnotation();
    annotation = !annotation;
    annotationToggle.attr('color', annotation ? null : 'lightgray');
  });

  return annotationControls;
}
