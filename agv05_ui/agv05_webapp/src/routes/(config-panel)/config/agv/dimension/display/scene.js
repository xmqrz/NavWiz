/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import gridF from 'mapx-layout-editor/scene/grid';

export default function (viz, payloadIdx) {
  /**
   * Layer: Scene
   * Scene objects are subjected to the effect of panning, zooming and rotation.
   */
  var scene = viz.append('g').attr('class', 'scene');
  var grid = gridF(viz, scene);
  var bbox = scene.append('g').attr('class', 'bbox');
  var robot = scene.append('g').attr('class', 'robot');
  var sprite = robot.append('path');
  var payload = robot.append('g').attr('class', 'payload');
  var payloadSprite = payload.append('path');
  var dims = scene.append('g').attr('class', 'dims');

  scene.zoomed = function () {
    scene.attr('transform', `translate(${viz.meta.translate})scale(${viz.meta.scale})`);
    grid.zoomed();
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    var body = viz.models.body();
    sprite.attrs({
      d: body.svgPath
    });

    if (payloadIdx) {
      var payload = viz.models.payload(payloadIdx - 1);
      payloadSprite.attrs({
        d: payload.svgPath
      });
      updatePayloadDims(payload);
    } else {
      updateBodyDims(body);
    }
  };

  function updateBodyDims(body) {
    if (!body.length || !body.width) {
      bbox.attr('display', 'none');
      dims.attr('display', 'none');
      return;
    }
    bbox.attr('display', 'block');
    dims.attr('display', 'block');

    var top = body.vcenter - body.length;
    var bottom = body.vcenter;
    var left = -body.hcenter;
    var right = body.width - body.hcenter;
    var bboxLines = [
      [-10, top, 10, top],
      [-10, bottom, 10, bottom],
      [left, -10, left, 10],
      [right, -10, right, 10],
      [-10, 0, 10, 0],
      [0, -10, 0, 10]
    ];
    var b = bbox.selectAll('line').data(bboxLines).join('line');
    b.attrs({
      x1: (d) => d[0],
      y1: (d) => d[1],
      x2: (d) => d[2],
      y2: (d) => d[3]
    });
    // b.exit().remove();

    var dimsLines = [
      [right + 0.08, top, right + 0.08, bottom],
      [left, bottom + 0.08, right, bottom + 0.08]
    ];
    var dimsTexts = [
      ['Length', right + 0.2, (top + bottom) / 2, -90],
      ['Width', (left + right) / 2, bottom + 0.2, 0]
    ];
    if (body.template !== 'custom') {
      dimsLines.push([left - 0.08, 0, left - 0.08, bottom]);
      dimsTexts.push(['Center of Rotation', left - 0.14, bottom / 2, -90]);
    } else {
      dimsTexts[0][0] = `Length: ${body.length}m`;
      dimsTexts[1][0] = `Width: ${body.width}m`;
    }

    var l = dims.selectAll('line').data(dimsLines).join('line');
    l.attrs({
      'marker-start': 'url(#arrow)',
      'marker-end': 'url(#arrow)'
    });
    l.attrs({
      x1: (d) => d[0],
      y1: (d) => d[1],
      x2: (d) => d[2],
      y2: (d) => d[3]
    });
    // l.exit().remove();

    var t = dims.selectAll('text').data(dimsTexts).join('text').attr('text-anchor', 'middle');
    t.text((d) => d[0]).attrs({
      transform: (d) => `translate(${d[1]},${d[2]})rotate(${d[3]})`
    });
    // t.exit().remove();
  }

  function updatePayloadDims(payload) {
    if (!payload.length || !payload.width) {
      bbox.attr('display', 'none');
      dims.attr('display', 'none');
      return;
    }
    bbox.attr('display', 'block');
    dims.attr('display', 'block');

    var top = payload.vcenter - payload.length / 2;
    var bottom = payload.vcenter + payload.length / 2;
    var left = payload.hcenter - payload.width / 2;
    var right = payload.hcenter + payload.width / 2;
    var bboxLines = [
      [-10, top, 10, top],
      [-10, bottom, 10, bottom],
      [left, -10, left, 10],
      [right, -10, right, 10]
    ];
    var b = bbox.selectAll('line').data(bboxLines).join('line');
    b.attrs({
      x1: (d) => d[0],
      y1: (d) => d[1],
      x2: (d) => d[2],
      y2: (d) => d[3]
    });
    // b.exit().remove();

    var dimsLines = [
      [right + 0.08, top, right + 0.08, bottom],
      [left, bottom + 0.08, right, bottom + 0.08]
    ];
    var dimsTexts = [
      ['Length', right + 0.2, (top + bottom) / 2, -90],
      ['Width', (left + right) / 2, bottom + 0.2, 0]
    ];
    if (payload.template === 'custom') {
      dimsTexts[0][0] = `Length: ${payload.length}m`;
      dimsTexts[1][0] = `Width: ${payload.width}m`;
    }

    var l = dims.selectAll('line').data(dimsLines).join('line').attrs({
      'marker-start': 'url(#arrow)',
      'marker-end': 'url(#arrow)'
    });
    l.attrs({
      x1: (d) => d[0],
      y1: (d) => d[1],
      x2: (d) => d[2],
      y2: (d) => d[3]
    });
    // l.exit().remove();

    var t = dims.selectAll('text').data(dimsTexts).join('text').attr('text-anchor', 'middle');
    t.text((d) => d[0]).attrs({
      transform: (d) => `translate(${d[1]},${d[2]})rotate(${d[3]})`
    });
    // t.exit().remove();
  }

  return scene;
}
