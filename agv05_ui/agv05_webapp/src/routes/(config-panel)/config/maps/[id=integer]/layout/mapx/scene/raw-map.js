/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene) {
  /* Raw Map */
  var rawMap = scene.append('g').attr('class', 'raw-map');
  var png = rawMap.append('image');
  var shown = true;

  function modelsUpdated() {
    var ocg = viz.models.ocg();
    if (!ocg) {
      clearRawMap();
    } else {
      try {
        updateRawMap(JSON.parse(ocg.metadata), ocg.url);
      } catch (e) {
        console.log('Map ocg metadata parse error.');
      }
    }
  }

  function updateRawMap(data, url) {
    // offset by half a pixel
    var offX = data.x0 - data.resolution / 2;
    var offY = data.y0 - data.resolution / 2;

    png
      .attr('xlink:href', url ? url : 'data:image/png;base64,' + data.png)
      .attr('width', data.width)
      .attr('height', data.height)
      .attr('transform', `translate(${offX},${offY})scale(${data.resolution})`);
  }

  function clearRawMap() {
    png.attr('xlink:href', null);
  }

  function toggle() {
    shown = !shown;
    rawMap.style('display', shown ? 'block' : 'none');
  }

  return {
    modelsUpdated: modelsUpdated,
    updateRawMap: updateRawMap,
    clearRawMap: clearRawMap,
    toggle: toggle
  };
}
