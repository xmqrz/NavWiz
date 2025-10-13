/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as fflate from 'fflate';
import modelsF from 'mapx-layout-editor/models';

export default function (viz) {
  // Inherit from mapx models
  modelsF(viz);

  /* Model for qmap */
  var qmap = {};
  var metadata;

  viz.models.loadQmap = function (options) {
    qmap = {};
    metadata = options.metadata;

    if (options.png && metadata) {
      metadata.colorType = 3;
      if (options.png.avg) {
        try {
          qmap.avg = decodePng(options.png.avg, metadata);
        } catch (e) {
          console.log('Fail to decode qmap (avg):', e);
        }
      }
      if (options.png.min) {
        try {
          qmap.min = decodePng(options.png.min, metadata);
        } catch (e) {
          console.log('Fail to decode qmap (min):', e);
        }
      }
      if (options.png.max) {
        try {
          qmap.max = decodePng(options.png.max, metadata);
        } catch (e) {
          console.log('Fail to decode qmap (max):', e);
        }
      }

      metadata.colorType = 6;
      if (options.png.count) {
        try {
          qmap.count = decodePng(options.png.count, metadata);
        } catch (e) {
          console.log('Fail to decode qmap (count):', e);
        }
      }
    }

    viz.scene.updateQmap(metadata, options.url);
  };

  viz.models.clearQmap = function () {
    qmap = {};
    viz.scene.clearQmap();
  };

  viz.models.qmapPixel = function (x, y) {
    var pixelIndex = y * metadata.width + x;
    return {
      avg: qmap.avg ? qmap.avg[pixelIndex] : '-',
      min: qmap.min ? qmap.min[pixelIndex] : '-',
      max: qmap.max ? qmap.max[pixelIndex] : '-',
      count: qmap.count ? qmap.count[pixelIndex] : '-'
    };
  };
}

// Reference: https://github.com/vivaxy/png/blob/master/src/decode/index.ts
function decodePng(arrayBuffer, metadata) {
  var buf = new Uint8Array(arrayBuffer);
  var idat = new Uint8Array();
  var index = 0;

  // Helpers
  function readUInt32BE() {
    return (buf[index++] << 24) | (buf[index++] << 16) | (buf[index++] << 8) | buf[index++];
  }

  function readUInt8() {
    return buf[index++];
  }

  function readChunkType() {
    let name = '';
    for (const end = index + 4; index < end; index++) {
      name += String.fromCharCode(buf[index]);
    }
    return name;
  }

  // Signature
  const PNG_SIGNATURE = [0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a];
  for (; index < PNG_SIGNATURE.length; ++index) {
    if (buf[index] !== PNG_SIGNATURE[index]) {
      throw new Error('Invalid file signature');
    }
  }

  // Chunks
  function parseIHDR() {
    if (metadata.width !== readUInt32BE() || metadata.height !== readUInt32BE()) {
      throw new Error('Dimension mismatch');
    }
    var depth = readUInt8();
    if (depth !== 8) {
      throw new Error('Unsupported depth: ' + depth);
    }
    var colorType = readUInt8(); // bits: 1 palette, 2 color, 4 alpha
    if (metadata.colorType !== colorType) {
      throw new Error('Color type mismatch: ' + colorType);
    }
    var _compression = readUInt8();
    var _filter = readUInt8();
    var interlace = readUInt8();
    if (interlace !== 0) {
      throw new Error('Unsupported interlace: ' + interlace);
    }
  }

  function parseIDAT(length) {
    // save data, decode later
    idat = concatUInt8Array(idat, new Uint8Array(buf.buffer, index, length));
    index += length;
  }

  while (index < buf.length) {
    let length = readUInt32BE();
    let type = readChunkType();

    if (type === 'IHDR') {
      parseIHDR();
    } else if (type === 'IDAT') {
      parseIDAT(length);
    } else {
      index += length;
    }
    index += 4; // skip crc
  }

  // decode IDAT
  var inflatedData = fflate.decompressSync(idat);
  var bytePerPixel = metadata.colorType === 6 ? 4 : 1;
  var pixels = new Uint8Array(metadata.width * bytePerPixel * metadata.height);

  var dataIndex = 0;
  var prevUnfilteredLine = new Uint8Array();

  for (var y = 0; y < metadata.height; ++y) {
    var scanlineWidth = metadata.width * bytePerPixel + 1; // 1 byte for filter type

    // unfilter
    var filterType = inflatedData[dataIndex];
    if (!(filterType in unfilters)) {
      throw new Error('Unsupported filter type: ' + filterType);
    }
    var unfilteredLine = unfilters[filterType](
      new Uint8Array(inflatedData.buffer, dataIndex + 1, scanlineWidth - 1),
      new Uint8Array(pixels.buffer, metadata.width * bytePerPixel * y, scanlineWidth - 1),
      bytePerPixel,
      prevUnfilteredLine
    );
    prevUnfilteredLine = unfilteredLine;
    dataIndex += scanlineWidth;
  }

  if (metadata.colorType === 6) {
    pixels = new Uint32Array(pixels.buffer);
  }

  return pixels;
}

var unfilters = {
  // NONE
  0: function (data, unfilteredLine) {
    unfilteredLine.set(data);
    return unfilteredLine;
  },

  // SUB
  1: function (data, unfilteredLine, bytePerPixel) {
    for (var i = 0; i < data.length; ++i) {
      var left = unfilteredLine[i - bytePerPixel] || 0;
      unfilteredLine[i] = data[i] + left;
    }
    return unfilteredLine;
  },

  // UP
  2: function (data, unfilteredLine, bytePerPixel, prevUnfilteredLine) {
    for (var i = 0; i < data.length; ++i) {
      var above = prevUnfilteredLine[i] || 0;
      unfilteredLine[i] = data[i] + above;
    }
    return unfilteredLine;
  },

  // AVERAGE
  3: function (data, unfilteredLine, bytePerPixel, prevUnfilteredLine) {
    for (var i = 0; i < data.length; ++i) {
      var left = unfilteredLine[i - bytePerPixel] || 0;
      var above = prevUnfilteredLine[i] || 0;
      var avg = (left + above) >> 1;
      unfilteredLine[i] = data[i] + avg;
    }
    return unfilteredLine;
  },

  // PAETH
  4: function (data, unfilteredLine, bytePerPixel, prevUnfilteredLine) {
    for (var i = 0; i < data.length; ++i) {
      var left = unfilteredLine[i - bytePerPixel] || 0;
      var above = prevUnfilteredLine[i] || 0;
      var upperLeft = prevUnfilteredLine[i - bytePerPixel] || 0;
      var p = paeth(left, above, upperLeft);
      unfilteredLine[i] = data[i] + p;
    }
    return unfilteredLine;
  }
};

function paeth(left, above, upperLeft) {
  var p = left + above - upperLeft;
  var pa = Math.abs(p - left);
  var pb = Math.abs(p - above);
  var pc = Math.abs(p - upperLeft);
  if (pa <= pb && pa <= pc) {
    return left;
  } else if (pb <= pc) {
    return above;
  } else {
    return upperLeft;
  }
}

function concatUInt8Array(a, b) {
  var concated = new Uint8Array(a.length + b.length);
  concated.set(a);
  concated.set(b, a.length);
  return concated;
}
