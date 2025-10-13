/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import Changeset from './changeset';
import Junction from './junction';
import NoRotateZone from './no-rotate-zone';
import LocHintZone from './loc-hint-zone';
import Param from '$lib/shared/models/param.js';
import Path from './path';
import Station from './station';
import hitTest from './hit-test';

const MAX_UNDO_LEVELS = 150;

export default function (viz) {
  /* Model for map data */
  var $viz = $(viz.node());
  var junctions = [];
  var paths = [];
  var stations = [];
  var noRotateZones = [];
  var locHintZones = [];
  var params = [];
  var undoBuffer = [];
  var redoBuffer = [];
  var changesets = [];
  var dirty = false;
  var search;

  /* Deserialize from database */
  function load(options) {
    if (options.structure) {
      if (options.structure.junctions && Array.isArray(options.structure.junctions)) {
        junctions = _.map(options.structure.junctions, (j) => new Junction(j));
      }
      if (options.structure.paths && Array.isArray(options.structure.paths)) {
        paths = _.map(options.structure.paths, function (path) {
          // For backward compatibility. Migrate from 'path.speedLimit' to 'path.speed'.
          if ('speedLimit' in path) {
            if (path.speedLimit > 0) {
              path.speed = path.speedLimit / 60.0;
            } else {
              path.speed =
                ['Unlimited', 'Medium Fast', 'Medium', 'Slow'][-path.speedLimit] ||
                'Unlimited';
              path.speed = `\${${path.speed}}`;
            }
          }
          return new Path(path);
        });
      }
      if (options.structure.stations && Array.isArray(options.structure.stations)) {
        stations = _.map(options.structure.stations, (s) => new Station(s));
      }
      if (
        options.structure.no_rotate_zones &&
        Array.isArray(options.structure.no_rotate_zones)
      ) {
        noRotateZones = _.filter(
          _.map(options.structure.no_rotate_zones, (n) => new NoRotateZone(n)),
          (zone) => zone.points && zone.points.length && zone.points.length >= 3
        );
      }
      if (
        options.structure.loc_hint_zones &&
        Array.isArray(options.structure.loc_hint_zones)
      ) {
        locHintZones = _.filter(
          _.map(options.structure.loc_hint_zones, (n) => new LocHintZone(n)),
          (zone) => zone.points && zone.points.length && zone.points.length >= 3
        );
      }
    }
    if (options.metadata) {
      // For backward compatibility only. We have since switched to using 'options.structure'
      // to store all junctions' data.
      if (options.metadata.junctions && Array.isArray(options.metadata.junctions)) {
        junctions = _.map(options.metadata.junctions, (j) => new Junction(j));
      }
    }
    params = [];
    if (options.reservedParams && Array.isArray(options.reservedParams)) {
      params = params.concat(_.map(options.reservedParams, (p) => new Param(p)));
    }
    if (options.params && Array.isArray(options.params)) {
      params = params.concat(_.map(options.params, (p) => new Param(p)));
    }

    if (options.search) {
      search = options.search;
      // Handle if pass resource with prefix.
      let prefix = '_Station_';
      if (search.startsWith(prefix)) {
        search = search.substring(prefix.length);
      }
    }

    viz.modelsUpdated();
    window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  function loadChangesets(options) {
    if (options.changesets && Array.isArray(options.changesets)) {
      changesets = _.map(options.changesets, (c) => new Changeset(c));
    }
    viz.modelsUpdated();
    $viz.trigger('models.changesetsUpdated');
  }

  /* Serialize to database */
  function getMetadata() {
    return {};
  }

  function getStructure() {
    return {
      junction_count: junctions.length,
      junctions: _.map(junctions, (junction) => _.pick(junction, Junction.__attrs__)),
      paths: _.map(paths, (path) => _.pick(path, Path.__attrs__)),
      stations: _.map(stations, (station) => _.pick(station, Station.__attrs__)),
      no_rotate_zones: _.map(noRotateZones, (zone) => _.pick(zone, NoRotateZone.__attrs__)),
      loc_hint_zones: _.map(locHintZones, (zone) => _.pick(zone, LocHintZone.__attrs__))
    };
  }

  function getStations() {
    return _.map(stations, 'name');
  }

  /* Helper functions */
  function computeBoundingBox() {
    if (!junctions.length && !noRotateZones.length && !locHintZones.length) {
      return null;
    }
    var bbox = {
      top: 10000,
      bottom: -10000,
      left: 10000,
      right: -10000
    };
    for (let junction of junctions) {
      if (junction.x < bbox.left) {
        bbox.left = junction.x;
      }
      if (junction.x > bbox.right) {
        bbox.right = junction.x;
      }
      if (junction.y < bbox.top) {
        bbox.top = junction.y;
      }
      if (junction.y > bbox.bottom) {
        bbox.bottom = junction.y;
      }
    }
    for (let zone of noRotateZones) {
      for (let pt of zone.points) {
        if (pt[0] < bbox.left) {
          bbox.left = pt[0];
        }
        if (pt[0] > bbox.right) {
          bbox.right = pt[0];
        }
        if (pt[1] < bbox.top) {
          bbox.top = pt[1];
        }
        if (pt[1] > bbox.bottom) {
          bbox.bottom = pt[1];
        }
      }
    }
    for (let zone of locHintZones) {
      for (let pt of zone.points) {
        if (pt[0] < bbox.left) {
          bbox.left = pt[0];
        }
        if (pt[0] > bbox.right) {
          bbox.right = pt[0];
        }
        if (pt[1] < bbox.top) {
          bbox.top = pt[1];
        }
        if (pt[1] > bbox.bottom) {
          bbox.bottom = pt[1];
        }
      }
    }
    return bbox;
  }

  function lookupConnections(j_list) {
    if (!Array.isArray(j_list)) {
      j_list = [j_list];
    }
    if (j_list.some((j) => junctions.indexOf(j) < 0)) {
      return false;
    }

    var pp = [];
    for (let path of paths) {
      let j1 = junctions[path.j1];
      let j2 = junctions[path.j2];
      if (
        (j_list.indexOf(j1) >= 0 && j_list.indexOf(j2) < 0) ||
        (j_list.indexOf(j2) >= 0 && j_list.indexOf(j1) < 0)
      ) {
        pp.push([j1, j2, path]);
      }
    }
    return pp;
  }

  function lookupPathsFromJunctions(j_list) {
    return paths.filter(function (path) {
      let j1 = junctions[path.j1];
      let j2 = junctions[path.j2];
      return j_list.indexOf(j1) >= 0 && j_list.indexOf(j2) >= 0;
    });
  }

  function lookupStationsFromJunction(junction) {
    var j;
    if (Number.isInteger(junction)) {
      j = junction;
    } else {
      j = junctions.indexOf(junction);
    }
    if (j < 0 || j >= junctions.length) {
      console.log(
        'Attempting to lookup station from non junction. Possibly confusing results.'
      );
      return false;
    }
    var found = [];
    for (let station of stations) {
      if (station.j === j) {
        found.push(station);
      }
    }
    found.sort((s1, s2) => s1.direction - s2.direction);
    return found;
  }

  function lookupJunctionsFromObjects(obs) {
    var found = [];
    obs.forEach(function (ob) {
      if (ob instanceof Junction) {
        if (found.indexOf(ob) < 0) {
          found.push(ob);
        }
      } else if (ob instanceof Station) {
        let j = junctions[ob.j];
        if (found.indexOf(j) < 0) {
          found.push(j);
        }
      } else if (ob instanceof Path) {
        let j1 = junctions[ob.j1];
        let j2 = junctions[ob.j2];
        if (found.indexOf(j1) < 0) {
          found.push(j1);
        }
        if (found.indexOf(j2) < 0) {
          found.push(j2);
        }
      }
    });
    return found;
  }

  /* Manipulation: Junction */
  function addJunction(junction) {
    var idx = junctions.indexOf(junction);
    if (idx >= 0) {
      return idx;
    }
    junctions.push(new Junction(junction));
    return junctions.length - 1;
  }

  function updateJunction(junction) {
    var idx = junctions.indexOf(junction);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeJunction(junction) {
    var idx;
    if (Number.isInteger(junction)) {
      idx = junction;
    } else {
      idx = junctions.indexOf(junction);
    }
    if (idx < 0 || idx >= junctions.length) {
      return;
    }

    junctions.splice(idx, 1);

    // Adjust junction indexes.
    for (let path of paths) {
      if (path.j1 > idx) {
        --path.j1;
      }
      if (path.j2 > idx) {
        --path.j2;
      }
    }

    for (let station of stations) {
      if (station.j > idx) {
        --station.j;
      }
    }
  }

  function isJunctionMergeAllowed(junction1, junction2) {
    var idx1 = junctions.indexOf(junction1);
    var idx2 = junctions.indexOf(junction2);
    if (idx1 < 0 || idx2 < 0) {
      return false;
    }
    var indexes = [idx1, idx2];
    var stationsDirection = {};
    var hasStation = false;
    for (let station of stations) {
      // check if station overlap between j1 and j2
      if (indexes.includes(station.j)) {
        if (stationsDirection[station.direction]) {
          return false;
        }
        stationsDirection[station.direction] = true;
        hasStation = true;
      }
    }

    if (!hasStation) {
      return true;
    }

    // ensure j1 and j2 are connected by more than a single common path,
    // which would be collapsed by the merge
    var ref = 0;
    for (let path of paths) {
      if (indexes.includes(path.j1) || indexes.includes(path.j2)) {
        if (ref) {
          return true;
        }
        ++ref;
      }
    }
    return false;
  }

  /* Manipulation: Path */
  function addPath(path, junction1, junction2, pushHistory = true) {
    if (pushHistory) {
      push('addPath');
    }
    var idx = paths.indexOf(path);
    if (idx >= 0) {
      return idx;
    }
    path.j1 = addJunction(junction1);
    path.j2 = addJunction(junction2);
    let newPath = new Path(path);
    paths.push(newPath);
    updateBranch(newPath);
    triggerDirty();
    return paths.length - 1;
  }

  function updatePath(path) {
    var idx = paths.indexOf(path);
    if (idx >= 0) {
      updateBranch(path);
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removePath(path) {
    var idx = paths.indexOf(path);
    if (idx >= 0) {
      // Validate whether safe for removal.
      var removes = isPathRemovalAllowed(path);
      if (!removes) {
        return;
      }
      // Remove junction j1 & j2 if they are unreferenced elsewhere.
      if (removes[0]) {
        removeJunction(path.j1);
      }
      if (removes[1]) {
        removeJunction(path.j2);
      }
      removeBranch(path, idx);
      paths.splice(idx, 1);
      triggerDirty();
    }
  }

  function updateBranch(path) {
    /*
     *Update branching info (does not check conflict).
     */
    let idx = paths.indexOf(path);
    if (idx < 0) {
      return;
    }

    let bj1Updated = false;
    let bj2Updated = false;
    for (let testP of paths) {
      if (path === testP) {
        continue;
      }

      let isCommonPathJ1 = [testP.j1, testP.j2].indexOf(path.j1) >= 0;
      let isCommonPathJ2 = [testP.j1, testP.j2].indexOf(path.j2) >= 0;

      if (!isCommonPathJ1 && !isCommonPathJ2) {
        continue;
      } else if (isCommonPathJ1 && isCommonPathJ2) {
        continue;
      }
      let commonJ = isCommonPathJ1 ? path.j1 : path.j2;
      let pathD = isCommonPathJ1
        ? ['bj1', path.direction, path.j2]
        : ['bj2', Path.getReverseDirection(path), path.j1];
      let testD =
        testP.j1 === commonJ
          ? ['bj1', testP.direction, testP.j2]
          : ['bj2', Path.getReverseDirection(testP), testP.j1];

      //check is same direction
      if (pathD[1] !== testD[1]) {
        continue;
      }
      //remove previous branching
      removeBranchRelation(path, pathD[0]);
      removeBranchRelation(testP, testD[0]);

      //update new branching
      let testIdx = paths.indexOf(testP);
      path[pathD[0]] = testIdx;
      testP[testD[0]] = idx;

      if (isCommonPathJ1) {
        bj1Updated = true;
      } else {
        bj2Updated = true;
      }

      if (bj1Updated && bj2Updated) {
        break;
      }
    }

    if (!bj1Updated) {
      removeBranchRelation(path, 'bj1');
    }
    if (!bj2Updated) {
      removeBranchRelation(path, 'bj2');
    }
  }

  function removeBranchRelation(path, branchID) {
    let idx = paths.indexOf(path);
    if (idx >= 0 && path[branchID] >= 0) {
      let branchP = paths[path[branchID]];
      if (branchP && branchP.bj1 === idx) {
        branchP.bj1 = undefined;
      } else if (branchP && branchP.bj2 === idx) {
        branchP.bj2 = undefined;
      } else {
        console.log('Remove branch fail with no matching branch id.');
        return;
      }
      path[branchID] = undefined;
    }
  }

  function removeBranch(removeP, idx) {
    //remove branch relation
    if (removeP.bj1 >= 0) {
      let branchP = paths[removeP.bj1];
      if (branchP && branchP.j1 === removeP.j1) {
        branchP.bj1 = undefined;
      } else if (branchP && branchP.j2 === removeP.j1) {
        branchP.bj2 = undefined;
      } else {
        console.log('Fail to remove branch relation while removing path.');
      }
      removeP.bj1 = undefined;
    }
    if (removeP.bj2 >= 0) {
      let branchP = paths[removeP.bj2];
      if (branchP && branchP.j1 === removeP.j2) {
        branchP.bj1 = undefined;
      } else if (branchP && branchP.j2 === removeP.j2) {
        branchP.bj2 = undefined;
      } else {
        console.log('Fail to remove branch relation while removing path.');
      }
      removeP.bj2 = undefined;
    }

    //remap index of branch path.
    for (let path of paths) {
      if (path.bj1 > idx) {
        --path.bj1;
      }
      if (path.bj2 > idx) {
        --path.bj2;
      }
    }
  }

  function isPathRemovalAllowed(path, excludeObs = []) {
    // Returns either false or an array consisting two booleans,
    // which indicate whether to remove j1 and/or j2.
    var refJ1 = false;
    var refJ2 = false;
    for (let p of paths) {
      if (p === path) {
        continue;
      }
      if (excludeObs.indexOf(p) >= 0) {
        continue;
      }
      if ([p.j1, p.j2].indexOf(path.j1) >= 0) {
        refJ1 = true;
      }
      if ([p.j1, p.j2].indexOf(path.j2) >= 0) {
        refJ2 = true;
      }
    }

    var checkJunctions = [];
    if (!refJ1) {
      checkJunctions.push(path.j1);
    }
    if (!refJ2) {
      checkJunctions.push(path.j2);
    }
    if (checkJunctions.length) {
      for (let station of stations) {
        if (excludeObs.indexOf(station) >= 0) {
          continue;
        }
        if (checkJunctions.indexOf(station.j) >= 0) {
          // Cannot remove due to attached station.
          return false;
        }
      }
    }
    return [!refJ1, !refJ2];
  }

  /* Manipulation: Station */
  function addStation(station, junction, pushHistory = true) {
    if (pushHistory) {
      push('addStation');
    }
    var idx = stations.indexOf(station);
    if (idx >= 0) {
      return idx;
    }
    station.j = junctions.indexOf(junction);
    if (station.j < 0) {
      console.log("Invalid junction passed to 'addStation'.");
      return null;
    }
    stations.push(new Station(station));
    triggerDirty();
    return stations.length - 1;
  }

  function updateStation(station) {
    var idx = stations.indexOf(station);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeStation(station) {
    var idx = stations.indexOf(station);
    if (idx >= 0) {
      stations.splice(idx, 1);
      triggerDirty();
    }
  }

  /* Manipulation: No-Rotate Zone */
  function addNoRotateZone(noRotateZone) {
    push('addNoRotateZone');
    var idx = noRotateZones.indexOf(noRotateZone);
    if (idx >= 0) {
      return idx;
    }
    noRotateZones.push(new NoRotateZone(noRotateZone));
    triggerDirty();
    return noRotateZones.length - 1;
  }

  function updateNoRotateZone(noRotateZone) {
    var idx = noRotateZones.indexOf(noRotateZone);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeNoRotateZone(noRotateZone) {
    var idx;
    if (Number.isInteger(noRotateZone)) {
      idx = noRotateZone;
    } else {
      idx = noRotateZones.indexOf(noRotateZone);
    }
    if (idx < 0 || idx >= noRotateZones.length) {
      return;
    }
    noRotateZones.splice(idx, 1);
    triggerDirty();
  }

  /* Manipulation: Location Hint Zone */
  function addLocHintZone(locHintZone) {
    push('addLocHintZone');
    var idx = locHintZones.indexOf(locHintZone);
    if (idx >= 0) {
      return idx;
    }
    locHintZones.push(new LocHintZone(locHintZone));
    triggerDirty();
    return locHintZones.length - 1;
  }

  function updateLocHintZone(locHintZone) {
    var idx = locHintZones.indexOf(locHintZone);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeLocHintZone(locHintZone) {
    var idx;
    if (Number.isInteger(locHintZone)) {
      idx = locHintZone;
    } else {
      idx = locHintZones.indexOf(locHintZone);
    }
    if (idx < 0 || idx >= locHintZones.length) {
      return;
    }
    locHintZones.splice(idx, 1);
    triggerDirty();
  }

  /* Manupulation: Undo & Redo */
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
      junctions: JSON.stringify(junctions),
      paths: JSON.stringify(paths),
      stations: JSON.stringify(stations),
      noRotateZones: JSON.stringify(noRotateZones),
      locHintZones: JSON.stringify(locHintZones)
    });
    if (buffer.length > MAX_UNDO_LEVELS) {
      buffer.shift();
    }
  }

  function _restoreStructure(buffer) {
    var op = buffer.pop();
    junctions = _.map(JSON.parse(op.junctions), (j) => new Junction(j));
    paths = _.map(JSON.parse(op.paths), (p) => new Path(p));
    stations = _.map(JSON.parse(op.stations), (s) => new Station(s));
    noRotateZones = _.map(JSON.parse(op.noRotateZones), (n) => new NoRotateZone(n));
    locHintZones = _.map(JSON.parse(op.locHintZones), (n) => new LocHintZone(n));
    viz.scene.activeObject.set(null);
    triggerDirty();
  }

  function clearAll() {
    if (
      junctions.length ||
      paths.length ||
      stations.length ||
      noRotateZones.length ||
      locHintZones.length
    ) {
      push('clearAll');
      junctions = [];
      paths = [];
      stations = [];
      noRotateZones = [];
      locHintZones = [];
      viz.scene.activeObject.set(null);
      triggerDirty();
    }
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
    loadChangesets: loadChangesets,
    getMetadata: getMetadata,
    getStructure: getStructure,
    getStations: getStations,

    computeBoundingBox: computeBoundingBox,
    hitTestJunction: function (testJ, excludeJ) {
      return hitTest.junction(junctions, paths, testJ, excludeJ);
    },
    hitTestPath: function (testJ1, testJ2, testP, excludeJ) {
      return hitTest.path(junctions, paths, testJ1, testJ2, testP, excludeJ);
    },
    hitTestPoint: function (testPt) {
      return hitTest.point(junctions, paths, stations, testPt);
    },
    hitTestRect: function (testPt1, testPt2) {
      return hitTest.rect(
        junctions,
        paths,
        stations,
        testPt1,
        testPt2,
        viz.zoom.getSnapDistance() / 2
      );
    },
    hitTestFzPerimeter: function (fz, testZ) {
      return hitTest.regionPerimeter(fz.points, testZ, viz.zoom.getSnapDistance() / 2);
    },
    lineSegmentsIntersect: hitTest.lineSegmentsIntersect,
    lookupConnections: lookupConnections,
    lookupPathsFromJunctions: lookupPathsFromJunctions,
    lookupStationsFromJunction: lookupStationsFromJunction,
    lookupJunctionsFromObjects: lookupJunctionsFromObjects,

    updateJunction: updateJunction,
    removeJunction: removeJunction,
    isJunctionMergeAllowed: isJunctionMergeAllowed,

    addPath: addPath,
    updatePath: updatePath,
    removePath: removePath,
    isPathRemovalAllowed: isPathRemovalAllowed,

    updateBranch: updateBranch,

    addStation: addStation,
    updateStation: updateStation,
    removeStation: removeStation,

    addNoRotateZone: addNoRotateZone,
    updateNoRotateZone: updateNoRotateZone,
    removeNoRotateZone: removeNoRotateZone,

    addLocHintZone: addLocHintZone,
    updateLocHintZone: updateLocHintZone,
    removeLocHintZone: removeLocHintZone,

    canUndo: canUndo,
    canRedo: canRedo,
    undo: undo,
    redo: redo,
    push: push,

    clearAll: clearAll,
    triggerDirty: triggerDirty,
    externalTriggerDirty: externalTriggerDirty,

    rawJunctions: () => junctions,
    rawPaths: () => paths,
    rawStations: () => stations,
    rawNoRotateZones: () => noRotateZones,
    rawLocHintZones: () => locHintZones,
    params: () => params,
    rawChangesets: () => changesets,
    rawSearch: () => search,
    isDirty: () => dirty
  };
}
