/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import pzntg from 'pzntg';

import $perf from '$lib/shared/perf';
import Metadata from './metadata';

const MAX_UNDO_LEVELS = 60;

export default function (viz) {
  /* Model for ocg data */
  var $viz = $(viz.node());
  var metadata;
  var pngCanvas;
  var maxUndoLevels = MAX_UNDO_LEVELS;
  var undoBuffer = [];
  var redoBuffer = [];
  var dirty = false;
  var ocgId;

  /* Deserilize from database */
  function load(options) {
    metadata = new Metadata(options.metadata);
    var png;
    if (options.png) {
      try {
        png = _sanitizePng(options.png);
      } catch (e) {
        window.alert('Failed to load raw map: Browser memory error.');
      }
    }
    if (png) {
      if (metadata.width !== png.width || metadata.height !== png.height) {
        window.requestAnimationFrame(function () {
          $viz.trigger('models.metadataMismatch', png);
          window.alert(
            'The size of the existing raw map is not the same as the size of the canvas.\n\nPlease adjust the canvas size.'
          );
        });
      } else {
        pngCanvas = png;
        if (metadata.width > 5000 || metadata.height > 5000) {
          maxUndoLevels = 6;
          window.alert('Max undo levels reduced to 6 due to large map.');
        }
      }
    }
    if (options.ocgId) {
      ocgId = options.ocgId;
    }
    viz.modelsUpdated();
    window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  /* Serialize to database */
  function getMetadata() {
    return _.pick(metadata, Metadata.__attrs__);
  }

  function getPng() {
    return _serializePng(pngCanvas);
  }

  /* Helper functions */
  function computeBoundingBox() {
    return {
      top: metadata.x0 + metadata.width * metadata.resolution,
      bottom: metadata.x0,
      left: metadata.y0 + metadata.height * metadata.resolution,
      right: metadata.y0
    };
  }

  /* Manipulation: Metadata & PNG */
  function updateMetadata(m, png) {
    if (pngCanvas) {
      push('updateMetadata');
    }
    metadata = new Metadata(m);
    pngCanvas = _blitPng(png);
    if (maxUndoLevels === MAX_UNDO_LEVELS && (metadata.width > 5000 || metadata.height > 5000)) {
      maxUndoLevels = 6;
      window.alert('Max undo levels reduced to 6 due to large map.');
    }
    triggerDirty();
  }

  function updatePng(png) {
    push('updatePng');
    pngCanvas = _blitPng(png);
    triggerDirty();
  }

  function downloadPng() {
    return _serializePng(_flipPng(pngCanvas), false);
  }

  function uploadPng(png) {
    try {
      png = _sanitizePng(_flipPng(png));
    } catch (e) {
      window.alert('Failed to upload raw map: Browser memory error.');
      return;
    }
    if (metadata.width !== png.width || metadata.height !== png.height) {
      $viz.trigger('models.metadataMismatch', png);
      window.alert(
        'The size of the uploaded raw map is not the same as the size of the canvas.\n\nPlease adjust the canvas size.'
      );
      return;
    }
    push('uploadPng');
    pngCanvas = png;
    triggerDirty();
  }

  function _blitPng(png) {
    var canvas = document.createElement('canvas');
    canvas.width = png.width;
    canvas.height = png.height;

    var ctx = canvas.getContext('2d');
    ctx.drawImage(png, 0, 0);
    return canvas;
  }

  function _flipPng(png) {
    // Flip the png from ROS coordinate frame to the normal display coordinate frame, or vice versa.
    var canvas = document.createElement('canvas');
    canvas.width = png.height;
    canvas.height = png.width;

    var ctx = canvas.getContext('2d');
    ctx.save();
    ctx.translate(canvas.width, canvas.height);
    ctx.scale(1, -1);
    ctx.rotate(Math.PI / 2);
    ctx.drawImage(png, 0, 0);
    ctx.restore();
    return canvas;
  }

  function _sanitizePng(png) {
    // Discard non-grayscale pixels as transparent pixels, and
    // limit the grayscale, except black and white, to the range 101-199.
    $perf.begin('_sanitizePng');
    var canvas = document.createElement('canvas');
    canvas.width = png.width;
    canvas.height = png.height;

    var ctx = canvas.getContext('2d');
    ctx.drawImage(png, 0, 0);

    var imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    var data = imageData.data;
    var idx = -1;
    var i, j, r, g, b, a;
    for (i = 0; i < imageData.height; ++i) {
      for (j = 0; j < imageData.width; ++j) {
        r = data[++idx];
        g = data[++idx];
        b = data[++idx];
        a = data[++idx];

        if (r === g && r === b && a === 255) {
          if (r > 0 && r <= 100) {
            idx -= 4;
            data[++idx] = 101;
            data[++idx] = 101;
            data[++idx] = 101;
            ++idx;
          } else if (r >= 200 && r < 255) {
            idx -= 4;
            data[++idx] = 199;
            data[++idx] = 199;
            data[++idx] = 199;
            ++idx;
          }
        } else {
          idx -= 4;
          data[++idx] = 0;
          data[++idx] = 0;
          data[++idx] = 0;
          data[++idx] = 0;
        }
      }
    }
    ctx.putImageData(imageData, 0, 0);
    $perf.end('_sanitizePng');
    return canvas;
  }

  function _serializePng(canvas, asObjectURL) {
    // convert the canvas to a PNG image, encoded in data url base64 format.
    $perf.begin('_serializePng');
    var ctx = canvas.getContext('2d');
    var imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    var data = imageData.data;
    var pixels = [];
    var idx = -1;
    var i, j, r, g, b, a, rowPixels;
    for (i = 0; i < imageData.height; ++i) {
      rowPixels = [];
      for (j = 0; j < imageData.width; ++j) {
        r = data[++idx];
        g = data[++idx];
        b = data[++idx];
        a = data[++idx];

        if (r === g && r === b && a === 255) {
          if (r === 255) {
            rowPixels.push(0);
          } else if (r === 0) {
            rowPixels.push(100);
          } else if (r >= 200) {
            rowPixels.push(1);
          } else if (r <= 100) {
            rowPixels.push(99);
          } else {
            rowPixels.push(200 - r);
          }
        } else {
          rowPixels.push(255); // set as unknown space
        }
      }
      pixels.push(rowPixels);
    }

    var palette = [];
    palette.push([255, 255, 255, 255]);
    for (i = 1; i < 100; ++i) {
      g = 200 - i;
      palette.push([g, g, g, 255]);
    }
    palette.push([0, 0, 0, 255]);
    for (i = 101; i < 256; ++i) {
      palette.push([0, 0, 0, 0]);
    }

    $perf.begin('pzntg');
    var png = pzntg.create({
      pixels: pixels,
      palette: palette,
      asBase64: !asObjectURL,
      zlib_level: 6, // best tradeoff between compression size and time
      zlib_strategy: 0, // default strategy (compress better than RLE)
      zlib_memlevel: 9 // use maximum memory for optimal speed
    });
    $perf.end('pzntg');

    if (asObjectURL) {
      png = window.URL.createObjectURL(
        new Blob([png], {
          type: 'image/png'
        })
      );
    }
    $perf.end('_serializePng');
    return png;
  }

  /* Manipulation: Undo & Redo */
  var canUndo = () => !!undoBuffer.length;
  var canRedo = () => !!redoBuffer.length;

  function undo() {
    if (!canUndo()) {
      return;
    }
    _preserveStructure(redoBuffer, 'undo');
    _restoreStructure(undoBuffer);
    $viz.trigger('models.undoBuffer');
  }

  function redo() {
    if (!canRedo()) {
      return;
    }
    _preserveStructure(undoBuffer, 'redo');
    _restoreStructure(redoBuffer);
    $viz.trigger('models.undoBuffer');
  }

  function push(name) {
    _preserveStructure(undoBuffer, name);
    redoBuffer = [];
    $viz.trigger('models.undoBuffer');
  }

  function _preserveStructure(buffer, name) {
    buffer.push({
      name: name,
      metadata: new Metadata(metadata),
      pngCanvas: pngCanvas
    });
    if (buffer.length > maxUndoLevels) {
      buffer.shift();
    }
  }

  function _restoreStructure(buffer) {
    var op = buffer.pop();
    pngCanvas = op.pngCanvas;
    metadata = op.metadata;
    triggerDirty();
  }

  function triggerDirty() {
    externalTriggerDirty();
    viz.modelsUpdated();
  }

  function externalTriggerDirty() {
    if (!dirty) {
      dirty = true;
      $viz.trigger('models.dirty');
    }
  }

  viz.models = {
    load: load,
    getMetadata: getMetadata,
    getPng: getPng,

    computeBoundingBox: computeBoundingBox,

    updateMetadata: updateMetadata,

    updatePng: updatePng,
    downloadPng: downloadPng,
    uploadPng: uploadPng,

    canUndo: canUndo,
    canRedo: canRedo,
    undo: undo,
    redo: redo,

    externalTriggerDirty: externalTriggerDirty,

    metadata: () => metadata,
    png: () => pngCanvas,
    isDirty: () => dirty,
    ocgId: () => ocgId
  };
}
