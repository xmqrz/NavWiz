/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import hitTest from 'map-layout-editor/models/hit-test';

var _names = ['far', 'middle', 'near'];
var _defaultRegions = {
  near: [
    [0, 0],
    [300, 300],
    [-300, 300]
  ],
  middle: [
    [0, 0],
    [400, 400],
    [-400, 400]
  ],
  far: [
    [0, 0],
    [450, 450],
    [-450, 450]
  ]
};

function compareCoords(a, b) {
  var argA = Math.atan2(a[1], a[0]);
  var argB = Math.atan2(b[1], b[0]);
  return argA < argB ? -1 : argA > argB ? 1 : 0;
}

export default function (viz) {
  /* Model for laser area data */
  var $viz = $(viz.node());
  var dirty = false;
  var areaData = {};
  var activeOption = [-1, 0]; //lidar num, area num
  var lidarData = [
    {
      prefix: 'lidar_1',
      min_activation: 4,
      name: 'Primary Lidar Sensors',
      areas: [],
      topics: []
    },
    {
      prefix: 'lidar_2',
      min_activation: 4,
      name: 'Secondary Lidar Sensors',
      areas: [],
      topics: []
    },
    {
      prefix: 'lidar_3',
      min_activation: 4,
      name: 'Tertiary Lidar Sensors',
      areas: [],
      topics: []
    },
    {
      prefix: 'lidar_4',
      min_activation: 4,
      name: 'Quaternary Lidar Sensors',
      areas: [],
      topics: []
    },
    {
      prefix: 'lidar_5',
      min_activation: 4,
      name: 'Quinary Lidar Sensors',
      areas: [],
      topics: []
    }
  ];
  var lidarOptions = [];
  var regionClipboard;

  /* Deserialize from database */
  function load(options) {
    lidarData[0].topics = options.lidar_1_topics || [];
    lidarData[1].topics = options.lidar_2_topics || [];
    lidarData[2].topics = options.lidar_3_topics || [];
    lidarData[3].topics = options.lidar_4_topics || [];
    lidarData[4].topics = options.lidar_5_topics || [];

    for (let areaName in options.configs) {
      // get min activation
      let suffix = 'min_activation_';
      if (areaName.endsWith(suffix)) {
        loadMinActivation(areaName, options.configs[areaName]);
        continue;
      }

      let regions;

      try {
        regions = JSON.parse(options.configs[areaName]);
      } catch (e) {
        console.log(`Laser area ${areaName} parse error.`);
        regions = {};
      }

      let defaultRegions = _.cloneDeep(_defaultRegions);
      let hasDefault = 0;

      if (areaName in options.defaults) {
        let rawDefaultRegions;
        try {
          rawDefaultRegions = JSON.parse(options.defaults[areaName]);
        } catch (e) {
          console.log(`Default laser area ${areaName} parse error.`);
          rawDefaultRegions = {};
        }
        for (let n of _names) {
          if (_validateRegion(rawDefaultRegions[n])) {
            defaultRegions[n] = rawDefaultRegions[n];
            hasDefault += 1;
          }
        }
      }

      for (let n of _names) {
        if (!_validateRegion(regions[n])) {
          regions[n] = _.cloneDeep(defaultRegions[n]);
        }
      }

      for (let lidar of lidarData) {
        if (!areaName.startsWith(lidar.prefix)) {
          continue;
        }
        let name = areaName.replace(`${lidar.prefix}_`, '');
        let data = {
          rawName: areaName,
          name: _.startCase(name),
          regions: regions
        };
        if (hasDefault >= 3) {
          data.defaults = defaultRegions;
        }
        lidar.areas.push(data);
      }
    }

    lidarOptions = [];
    for (let i in lidarData) {
      let lidar = lidarData[i];
      lidar.areas.sort(function (a, b) {
        if (a.name === b.name) {
          return 0;
        }
        return parseInt(a.name.match(/\d+/)) > parseInt(b.name.match(/\d+/)) ? 1 : -1;
      });
      if (lidar.topics && lidar.topics.length > 0) {
        lidarOptions.push([parseInt(i), lidar.name]);
      }
    }

    if (activeOption[0] === -1 && lidarOptions.length > 0) {
      activeOption[0] = lidarOptions[0][0];
      areaData = lidarData[activeOption[0]].areas[activeOption[1]];
    }

    if (
      lidarOptions.length === 0 ||
      !options.configs ||
      Object.keys(options.configs).length === 0
    ) {
      viz.error.show(
        'Robot controller is not running or laser sensor was not available for this robot.\nPlease refresh the page after the robot controller is restarted.'
      );
      return;
    }

    $viz.trigger('models.loaded');
    viz.modelsUpdated();
    window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  function loadMinActivation(name, value) {
    for (let lidar of lidarData) {
      if (!name.startsWith(lidar.prefix)) {
        continue;
      }
      try {
        lidar.min_activation = Number.parseInt(value);
      } catch (e) {
        console.log(`Laser min activation ${name} parse error.`);
      }
    }
  }

  function getMinActivation() {
    if (activeOption[0] < 0) {
      return;
    }

    return lidarData[activeOption[0]].min_activation;
  }

  function _validateRegion(coords) {
    if (!coords || !Array.isArray(coords) || coords.length < 3) {
      return false;
    }
    for (let coord of coords) {
      if (!Array.isArray(coord) || coord.length !== 2) {
        return false;
      }
    }
    return true;
  }

  function setActiveLidar(lidarIndex) {
    if (lidarIndex === activeOption[0]) {
      return;
    }

    let lidar = lidarData[lidarIndex];
    if (!lidar || !lidar.topics || lidar.topics.length <= 0) {
      return;
    }

    activeOption[0] = lidarIndex;
    activeOption[1] = 0;
    areaData = lidarData[activeOption[0]].areas[activeOption[1]];

    $viz.trigger('models.loaded');
    viz.modelsUpdated();
  }

  function setActiveArea(areaIndex) {
    if (areaIndex === activeOption[1] || areaIndex < 0) {
      return;
    }
    let data = lidarData[activeOption[0]];
    if (!data || areaIndex >= data.areas.length) {
      return;
    }

    activeOption[1] = areaIndex;
    areaData = lidarData[activeOption[0]].areas[activeOption[1]];

    viz.modelsUpdated(); //no need to trigger dirty.
  }

  /* Helper functions */
  function computeBoundingBox() {
    var bbox = {
      top: -10000000,
      bottom: 10000000,
      left: -10000000,
      right: 10000000
    };
    for (let n of _names) {
      for (let coord of areaData.regions[n]) {
        if (coord[0] > bbox.top) {
          bbox.top = coord[0];
        }
        if (coord[0] < bbox.bottom) {
          bbox.bottom = coord[0];
        }
        if (coord[1] > bbox.left) {
          bbox.left = coord[1];
        }
        if (coord[1] < bbox.right) {
          bbox.right = coord[1];
        }
      }
    }
    // make symmetrical
    if (bbox.top < -bbox.bottom) {
      bbox.top = -bbox.bottom;
    } else {
      bbox.bottom = -bbox.top;
    }
    if (bbox.left < -bbox.right) {
      bbox.left = -bbox.right;
    } else {
      bbox.right = -bbox.left;
    }
    bbox.left *= 0.001;
    bbox.right *= 0.001;
    bbox.top *= 0.001;
    bbox.bottom *= 0.001;
    return bbox;
  }

  /* Manipulation: Region */
  function addCoord(region, coord, idx) {
    if (_names.indexOf(region) < 0) {
      return;
    }
    _snapCoordBoundary(coord);

    if (idx === undefined) {
      let pt;
      [pt, idx] = hitTest.regionPerimeter(
        areaData.regions[region],
        coord,
        (viz.zoom.getSnapDistance() / 2) * 1000
      );
      if (!pt) {
        return;
      }
    }

    let newRegion = areaData.regions[region].slice();
    newRegion.splice(idx, 0, coord);

    if (!_limitSimplePolygon(newRegion)) {
      return;
    }
    areaData.regions[region] = newRegion;
    triggerDirty();
    return idx;
  }

  function updateCoord(region, idx, xy, coord) {
    if (_names.indexOf(region) < 0 || idx < 0 || idx >= areaData.regions[region].length) {
      return;
    }
    if (Array.isArray(xy) && xy.length === 2) {
      // shift parameters to the left
      coord = xy;
    } else if (xy === 0) {
      coord = [coord, areaData.regions[region][idx][1]];
    } else if (xy === 1) {
      coord = [areaData.regions[region][idx][0], coord];
    } else {
      return;
    }

    _snapCoordBoundary(coord);

    let newRegion = areaData.regions[region].slice();
    newRegion[idx] = coord;
    if (!_limitSimplePolygon(newRegion)) {
      return;
    }

    areaData.regions[region] = newRegion;
    triggerDirty();
  }

  function removeCoord(region, idx) {
    if (
      _names.indexOf(region) < 0 ||
      idx < 0 ||
      idx >= areaData.regions[region].length ||
      areaData.regions[region].length <= 3
    ) {
      return;
    }
    let newRegion = areaData.regions[region].slice();
    newRegion.splice(idx, 1);
    if (!_limitSimplePolygon(newRegion)) {
      return false;
    }
    areaData.regions[region] = newRegion;
    triggerDirty();
  }

  function moveRegion(region, pt) {
    if (_names.indexOf(region) < 0) {
      return;
    }
    let firstPt = areaData.regions[region][0];
    let translate = [pt[0] - firstPt[0], pt[1] - firstPt[1]];
    let newRegion = [pt];
    for (let r of areaData.regions[region].slice(1)) {
      let newPt = [r[0] + translate[0], r[1] + translate[1]];
      if (!_limitCoordBoundary(newPt)) {
        return;
      }
      newRegion.push(newPt);
    }
    areaData.regions[region] = newRegion;
    triggerDirty();
  }

  function setMinActivation(min_activation) {
    if (activeOption[0] < 0) {
      return;
    }

    lidarData[activeOption[0]].min_activation = min_activation;
    triggerDirty();
  }

  function _limitCoordBoundary(coord) {
    coord[0] = Number.parseInt(coord[0]);
    coord[1] = Number.parseInt(coord[1]);

    if (Math.abs(coord[0]) > 7000 || Math.abs(coord[1]) > 7000) {
      return false;
    }
    return true;
  }

  function _snapCoordBoundary(coord) {
    coord[0] = Number.parseInt(coord[0]);
    coord[1] = Number.parseInt(coord[1]);

    if (coord[0] < -7000) {
      coord[0] = -7000;
    } else if (coord[0] > 7000) {
      coord[0] = 7000;
    }
    if (coord[1] < -7000) {
      coord[1] = -7000;
    } else if (coord[1] > 7000) {
      coord[1] = 7000;
    }
  }

  function _limitSimplePolygon(region) {
    if (region.length === 3) {
      var v1 = [region[1][0] - region[0][0], region[1][1] - region[0][1]];
      var v2 = [region[2][0] - region[0][0], region[2][1] - region[0][1]];
      return v1[0] * v2[1] - v1[1] * v2[0] !== 0; // cross product = 0 if parallel
    }
    for (let i = region.length - 1, j = 0; j < region.length - 2; i = j++) {
      for (let k = j + 1, l = j + 2; l < region.length; k = l++) {
        if (l === i) {
          continue;
        }
        if (hitTest.lineSegmentsIntersect(region[i], region[j], region[k], region[l])) {
          return false;
        }
      }
    }
    return true;
  }

  function resetToDefault(region) {
    if (_names.indexOf(region) < 0) {
      return;
    }
    areaData.regions[region] = _.cloneDeep(areaData.defaults[region]);
    triggerDirty();
  }

  function copyRegion(region) {
    if (_names.indexOf(region) < 0) {
      return;
    }
    regionClipboard = _.cloneDeep(areaData.regions[region]);
  }

  function pasteRegion(region) {
    if (!regionClipboard || _names.indexOf(region) < 0) {
      return;
    }
    areaData.regions[region] = _.cloneDeep(regionClipboard);
    triggerDirty();
  }

  function triggerDirty() {
    if (!dirty) {
      dirty = true;
      $viz.trigger('models.dirty');
    }
    viz.modelsUpdated();
  }

  function getParameters() {
    let config = {};
    for (let lidar of lidarData) {
      config[lidar.prefix + '_min_activation_'] = lidar.min_activation;
      for (let area of lidar.areas) {
        config[area.rawName] = JSON.stringify(area.regions);
      }
    }
    return config;
  }

  viz.models = {
    load: load,

    rawActiveOption: () => activeOption,
    rawLidarOptions: () => lidarOptions,
    getLidarData: (i) => lidarData[i],
    getMinActivation: getMinActivation,

    setActiveLidar: setActiveLidar,
    setActiveArea: setActiveArea,

    compareCoords: compareCoords,
    computeBoundingBox: computeBoundingBox,
    hitTestRegionPerimeter: function (region, pt) {
      if (!areaData) {
        return [false, false];
      }
      var poly = areaData.regions[region];
      return hitTest.regionPerimeter(
        poly,
        pt,
        (viz.zoom.getSnapDistance() / 2) * 1000,
        _limitCoordBoundary
      );
    },

    addCoord: addCoord,
    updateCoord: updateCoord,
    removeCoord: removeCoord,
    moveRegion: moveRegion,
    setMinActivation: setMinActivation,

    resetToDefault: resetToDefault,
    copyRegion: copyRegion,
    pasteRegion: pasteRegion,
    hasClipboard: () => !!regionClipboard,
    hasDefault: () => !!areaData.defaults,

    rawRegions: () => areaData.regions || {},
    rawScanId: () => (lidarData[activeOption[0]] || {}).topics,

    getParameters: getParameters,

    isDirty: () => dirty
  };
}
