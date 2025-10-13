/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Grid Lines */
  var gridSmall = scene.append('rect').attrs(
    Object.assign(
      {
        class: 'grid-small',
        style: 'fill:url("#grid-small-pattern")',
        display: 'none'
      },
      viz.meta.scene
    )
  );

  var grid = scene.append('rect').attrs(
    Object.assign(
      {
        class: 'grid',
        style: 'fill:url("#grid-pattern")'
      },
      viz.meta.scene
    )
  );

  grid.zoomed = function () {
    if (viz.zoom.getSnapDistance() === 0.2) {
      gridSmall.style('display', 'block');
    } else {
      gridSmall.style('display', 'none');
    }
  };

  return grid;
}
