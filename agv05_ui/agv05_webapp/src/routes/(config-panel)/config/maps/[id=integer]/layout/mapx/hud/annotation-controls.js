/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

export default function (viz, hud) {
  /* Annotation Controls */
  var annotationControls = hud.append('g').attr('class', 'annotation-controls');
  var annotationToggle = annotationControls.append('use').attrs({
    'xlink:href': '#annotation-toggle',
    'tip-title': 'Toggle annotation display',
    x: 26,
    y: -180
  });

  var annotation = true;

  annotationToggle.on('click', function (event) {
    event.stopPropagation();
    viz.scene.toggleAnnotation();
    annotation = !annotation;
    annotationToggle.attr('color', annotation ? null : 'lightgray');
  });

  return annotationControls;
}
