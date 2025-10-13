/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Grid Lines */
  var grid_01cm = scene.append('rect').attrs(
    Object.assign(
      {
        class: 'grid-1cm',
        style: 'fill:url("#grid-1cm-pattern")',
        display: 'none'
      },
      viz.meta.scene
    )
  );

  var grid_05cm = scene.append('rect').attrs(
    Object.assign(
      {
        class: 'grid-5cm',
        style: 'fill:url("#grid-5cm-pattern")',
        display: 'none'
      },
      viz.meta.scene
    )
  );

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
    if (viz.zoom.getSnapDistance() === 0.01) {
      grid_01cm.style('display', 'block');
      grid_05cm.style('display', 'block');
      gridSmall.style('display', 'block');
    } else if (viz.zoom.getSnapDistance() === 0.05) {
      grid_01cm.style('display', 'none');
      grid_05cm.style('display', 'block');
      gridSmall.style('display', 'block');
    } else if (viz.zoom.getSnapDistance() === 0.2) {
      grid_01cm.style('display', 'none');
      grid_05cm.style('display', 'none');
      gridSmall.style('display', 'block');
    } else {
      grid_01cm.style('display', 'none');
      grid_05cm.style('display', 'none');
      gridSmall.style('display', 'none');
    }
  };

  grid.show = function () {
    grid_01cm.style('visibility', 'visible');
    grid_05cm.style('visibility', 'visible');
    gridSmall.style('visibility', 'visible');
    grid.style('visibility', 'visible');
  };

  grid.hide = function () {
    grid_01cm.style('visibility', 'hidden');
    grid_05cm.style('visibility', 'hidden');
    gridSmall.style('visibility', 'hidden');
    grid.style('visibility', 'hidden');
  };

  return grid;
}
