/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import Changeset from 'map-layout-editor/models/changeset';
import ForbiddenZone from './forbidden-zone';
import Junction from './junction';
import Landmark from './landmark';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import Param from '$lib/shared/models/param.js';
import Path from './path';
import Station from './station';
import Layer from './layer';
import LocHintZone from 'map-layout-editor/models/loc-hint-zone';
import aligned from './aligned';
import branched from './branched';
import hitTest from './hit-test';
import hitTestMap from 'map-layout-editor/models/hit-test';

const MAX_UNDO_LEVELS = 150;

export default function (viz) {
  /* Model for map data */
  var $viz = $(viz.node());
  var ocg = 0;
  var ocgChoices = {};
  var junctions = [];
  var paths = [];
  var stations = [];
  var landmarks = [];
  var forbiddenZones = [];
  var noRotateZones = [];
  var locHintZones = [];
  var params = [];
  var undoBuffer = [];
  var redoBuffer = [];
  var changesets = [];
  var dirty = false;
  var search;
  var activeLayer = Layer.LAYER_0;
  var visibleLayers = {};
  Layer.list.forEach(([_, v]) => (visibleLayers[v] = true));

  /* Deserialize from database */
  function load(options) {
    if (options.ocgChoices && Array.isArray(options.ocgChoices)) {
      ocgChoices = _.keyBy(options.ocgChoices, 'id');
      if (options.ocgChoices.length) {
        ocg = options.ocgChoices[0].id;
      }
    }

    if (options.ocg) {
      ocg = options.ocg;
    }

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
      if (options.structure.landmarks && Array.isArray(options.structure.landmarks)) {
        landmarks = _.map(options.structure.landmarks, (l) => new Landmark(l));
      }
      if (
        options.structure.forbidden_zones &&
        Array.isArray(options.structure.forbidden_zones)
      ) {
        forbiddenZones = _.filter(
          _.map(options.structure.forbidden_zones, (f) => new ForbiddenZone(f)),
          (zone) => zone.points && zone.points.length && zone.points.length >= 3
        );
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
      landmarks: _.map(landmarks, (landmark) => _.pick(landmark, Landmark.__attrs__)),
      forbidden_zones: _.map(forbiddenZones, (zone) => _.pick(zone, ForbiddenZone.__attrs__)),
      no_rotate_zones: _.map(noRotateZones, (zone) => _.pick(zone, NoRotateZone.__attrs__)),
      loc_hint_zones: _.map(locHintZones, (zone) => _.pick(zone, LocHintZone.__attrs__))
    };
  }

  function getStations() {
    return _.map(stations, 'name');
  }

  /* Helper functions */
  function computeBoundingBox() {
    if (
      !junctions.length &&
      !landmarks.length &&
      !forbiddenZones.length &&
      !noRotateZones.length &&
      !locHintZones.length
    ) {
      return null;
    }
    var bbox = {
      top: -10000,
      bottom: 10000,
      left: -10000,
      right: 10000
    };
    for (let junction of junctions) {
      if (junction.x > bbox.top) {
        bbox.top = junction.x;
      }
      if (junction.x < bbox.bottom) {
        bbox.bottom = junction.x;
      }
      if (junction.y > bbox.left) {
        bbox.left = junction.y;
      }
      if (junction.y < bbox.right) {
        bbox.right = junction.y;
      }
    }
    for (let landmark of landmarks) {
      if (landmark.x > bbox.top) {
        bbox.top = landmark.x;
      }
      if (landmark.x < bbox.bottom) {
        bbox.bottom = landmark.x;
      }
      if (landmark.y > bbox.left) {
        bbox.left = landmark.y;
      }
      if (landmark.y < bbox.right) {
        bbox.right = landmark.y;
      }
    }
    for (let zone of forbiddenZones) {
      for (let pt of zone.points) {
        if (pt[0] > bbox.top) {
          bbox.top = pt[0];
        }
        if (pt[0] < bbox.bottom) {
          bbox.bottom = pt[0];
        }
        if (pt[1] > bbox.left) {
          bbox.left = pt[1];
        }
        if (pt[1] < bbox.right) {
          bbox.right = pt[1];
        }
      }
    }
    for (let zone of noRotateZones) {
      for (let pt of zone.points) {
        if (pt[0] > bbox.top) {
          bbox.top = pt[0];
        }
        if (pt[0] < bbox.bottom) {
          bbox.bottom = pt[0];
        }
        if (pt[1] > bbox.left) {
          bbox.left = pt[1];
        }
        if (pt[1] < bbox.right) {
          bbox.right = pt[1];
        }
      }
    }
    for (let zone of locHintZones) {
      for (let pt of zone.points) {
        if (pt[0] > bbox.top) {
          bbox.top = pt[0];
        }
        if (pt[0] < bbox.bottom) {
          bbox.bottom = pt[0];
        }
        if (pt[1] > bbox.left) {
          bbox.left = pt[1];
        }
        if (pt[1] < bbox.right) {
          bbox.right = pt[1];
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
    found.sort((s1, s2) => s1.heading - s2.heading);
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

  /* Manipulation: Ocg */
  function setOcg(idx) {
    if (idx in ocgChoices) {
      ocg = idx;
      triggerDirty();
    }
  }

  /* Manipulation: Junction */
  function addJunction(junction) {
    var idx = junctions.indexOf(junction);
    if (idx >= 0) {
      return idx;
    }
    junction.layer = activeLayer;
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
    var stationsHeading = {};
    var hasStation = false;
    for (let station of stations) {
      // check if station overlap between j1 and j2
      if (indexes.includes(station.j)) {
        if (stationsHeading[station.heading]) {
          return false;
        }
        stationsHeading[station.heading] = true;
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
    path.layer = activeLayer;

    let newPath = new Path(path);
    // reset branch when clone from branching path.
    newPath.bj1 = undefined;
    newPath.bj2 = undefined;

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

    if (!path.tracked) {
      removeBranchRelation(path, 'bj1');
      removeBranchRelation(path, 'bj2');
      return;
    }

    let forwardPathHeading = getPathHeading(path);
    let reversePathHeading = getPathReverseHeading(path);

    let bj1Updated = false;
    let bj2Updated = false;
    for (let testP of paths) {
      if (path === testP || !testP.tracked) {
        continue;
      }

      let isCommonPathJ1 = [testP.j1, testP.j2].indexOf(path.j1) >= 0;
      let isCommonPathJ2 = [testP.j1, testP.j2].indexOf(path.j2) >= 0;

      if (!isCommonPathJ1 && !isCommonPathJ2) {
        continue;
      } else if (isCommonPathJ1 && isCommonPathJ2) {
        continue;
      }

      // let pathJ1 = junctions[path.j1];
      // let pathJ2 = junctions[path.j2];
      // let testPJ1 = junctions[testP.j1];
      // let testPJ2 = junctions[testP.j2];

      let pathHeading = isCommonPathJ1 ? forwardPathHeading : reversePathHeading;
      if (pathHeading === undefined) {
        return;
      }

      let commonJ = isCommonPathJ1 ? path.j1 : path.j2;
      let testPHeading =
        testP.j1 === commonJ ? getPathHeading(testP) : getPathReverseHeading(testP);
      if (testPHeading === undefined) {
        return;
      }

      let pathD = isCommonPathJ1 ? 'bj1' : 'bj2';
      let testD = testP.j1 === commonJ ? 'bj1' : 'bj2';

      //check is same direction
      let headingDifference = Math.abs(pathHeading - testPHeading);
      headingDifference = Math.min(headingDifference, Math.abs(360 - headingDifference));
      if (headingDifference > 5.0) {
        continue;
      }

      //remove previous branching
      removeBranchRelation(path, pathD);
      removeBranchRelation(testP, testD);

      //update new branching
      let testIdx = paths.indexOf(testP);
      path[pathD] = testIdx;
      testP[testD] = idx;

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
    if (idx >= 0 && path[branchID] !== undefined && path[branchID] >= 0) {
      let branchP = paths[path[branchID]];
      if (!branchP) {
        console.log('Remove branch failed with no matching branch id.');
        return;
      }
      if (branchP && branchP.bj1 === idx) {
        findPathBranch(branchP, path, 'bj1');
      } else if (branchP && branchP.bj2 === idx) {
        findPathBranch(branchP, path, 'bj2');
      }
      path[branchID] = undefined;
    }
  }

  function removeBranch(removeP, idx) {
    //remove branch relation
    removeBranchRelation(removeP, 'bj1');
    removeBranchRelation(removeP, 'bj2');
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

  function findPathBranch(path, ignorePath, pathD) {
    let isCommonPathJ1 = pathD === 'bj1';
    let commonJ = isCommonPathJ1 ? path.j1 : path.j2;
    // let pathJ1 = junctions[path.j1];
    // let pathJ2 = junctions[path.j2];

    let pathHeading = isCommonPathJ1 ? getPathHeading(path) : getPathReverseHeading(path);
    if (pathHeading === undefined) {
      return;
    }

    for (let testP of paths) {
      if (testP === path || testP === ignorePath || !testP.tracked) {
        continue;
      }
      if ([testP.j1, testP.j2].indexOf(commonJ) < 0) {
        continue;
      }

      let testD = testP.j1 === commonJ ? 'bj1' : 'bj2';
      if (testP[testD] !== undefined) {
        continue;
      }

      let testPHeading =
        testP.j1 === commonJ ? getPathHeading(testP) : getPathReverseHeading(testP);
      if (testPHeading === undefined) {
        continue;
      }

      //check is same direction
      let headingDifference = Math.abs(pathHeading - testPHeading);
      headingDifference = Math.min(headingDifference, Math.abs(360 - headingDifference));
      if (headingDifference > 5.0) {
        continue;
      }

      path[pathD] = paths.indexOf(testP);
      testP[testD] = paths.indexOf(path);
      return;
    }

    path[pathD] = undefined;
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
    station.layer = activeLayer;
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

  /* Manipulation: Landmark */
  function addLandmark(landmark, pushHistory = true) {
    if (pushHistory) {
      push('addLandmark');
    }
    var idx = landmarks.indexOf(landmark);
    if (idx >= 0) {
      return idx;
    }
    landmarks.push(new Landmark(landmark));
    triggerDirty();
    return landmarks.length - 1;
  }

  function updateLandmark(landmark) {
    var idx = landmarks.indexOf(landmark);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeLandmark(landmark) {
    var idx;
    if (Number.isInteger(landmark)) {
      idx = landmark;
    } else {
      idx = landmarks.indexOf(landmark);
    }
    if (idx < 0 || idx >= landmarks.length) {
      return;
    }
    landmarks.splice(idx, 1);
    triggerDirty();
  }

  /* Manipulation: Forbidden Zone */
  function addForbiddenZone(forbiddenZone) {
    push('addForbiddenZone');
    var idx = forbiddenZones.indexOf(forbiddenZone);
    if (idx >= 0) {
      return idx;
    }
    forbiddenZones.push(new ForbiddenZone(forbiddenZone));
    triggerDirty();
    return forbiddenZones.length - 1;
  }

  function updateForbiddenZone(forbiddenZone) {
    var idx = forbiddenZones.indexOf(forbiddenZone);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeForbiddenZone(forbiddenZone) {
    var idx;
    if (Number.isInteger(forbiddenZone)) {
      idx = forbiddenZone;
    } else {
      idx = forbiddenZones.indexOf(forbiddenZone);
    }
    if (idx < 0 || idx >= forbiddenZones.length) {
      return;
    }
    forbiddenZones.splice(idx, 1);
    triggerDirty();
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
      landmarks: JSON.stringify(landmarks),
      forbiddenZones: JSON.stringify(forbiddenZones),
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
    landmarks = _.map(JSON.parse(op.landmarks), (l) => new Landmark(l));
    forbiddenZones = _.map(JSON.parse(op.forbiddenZones), (f) => new ForbiddenZone(f));
    noRotateZones = _.map(JSON.parse(op.noRotateZones), (n) => new NoRotateZone(n));
    locHintZones = _.map(JSON.parse(op.locHintZones), (n) => new LocHintZone(n));
    viz.scene.activeObject.set(null);
    triggerDirty();
  }

  function _getHeading(point1, point2) {
    let diffY = point2.y - point1.y;
    let diffX = point2.x - point1.x;
    let heading = (Math.atan2(diffY, diffX) * 180) / Math.PI;
    if (heading < 0) {
      heading += 360;
    }
    return heading;
  }

  function getPathHeading(path) {
    let point1 = junctions[path.j1];
    let point2;
    if (path.shape === Path.Shape.STRAIGHT) {
      point2 = junctions[path.j2];
    } else if (path.shape === Path.Shape.BEZIER) {
      point2 = path.cp1;
    } else {
      console.log('invalid path shape in get path heading');
      return;
    }
    return _getHeading(point1, point2);
  }

  function getPathReverseHeading(path) {
    let point1 = junctions[path.j2];
    let point2;
    if (path.shape === Path.Shape.STRAIGHT) {
      point2 = junctions[path.j1];
    } else if (path.shape === Path.Shape.BEZIER) {
      point2 = path.cp2;
    } else {
      console.log('invalid path shape in get path reverse heading');
      return;
    }
    return _getHeading(point1, point2);
  }

  function clearAll() {
    if (
      junctions.length ||
      paths.length ||
      stations.length ||
      landmarks.length ||
      forbiddenZones.length ||
      noRotateZones.length ||
      locHintZones.length
    ) {
      push('clearAll');
      junctions = [];
      paths = [];
      stations = [];
      landmarks = [];
      forbiddenZones = [];
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

  function setActiveLayer(layer) {
    if (!Number.isInteger(layer) || layer < Layer.LAYER_0 || layer > Layer.LAYER_NA) {
      return;
    }
    activeLayer = layer;
    visibleLayers[layer] = true;
  }

  function setVisibleLayer(layer, state) {
    if (!Number.isInteger(layer) || layer < Layer.LAYER_0 || layer > Layer.LAYER_NA) {
      return;
    }
    visibleLayers[layer] = !!state;
  }

  viz.models = {
    load: load,
    loadChangesets: loadChangesets,
    getMetadata: getMetadata,
    getStructure: getStructure,
    getStations: getStations,

    computeBoundingBox: computeBoundingBox,
    getAligned: function (ob) {
      return aligned.get(junctions, paths, stations, ob);
    },
    getBranched: function (ob) {
      return branched.get(junctions, paths, stations, ob);
    },
    hitTestJunction: function (testJ, excludeJ) {
      return hitTest.junction(
        junctions,
        paths,
        testJ,
        viz.zoom.getSnapDistance() / 2,
        null,
        excludeJ
      );
    },
    hitTestPath: function (testJ1, testJ2, testP, excludeJ) {
      return hitTest.path(
        junctions,
        paths,
        testJ1,
        testJ2,
        testP,
        viz.zoom.getSnapDistance() / 2,
        excludeJ
      );
    },
    selectObject: function (testPt) {
      return hitTest.selectObject(
        junctions,
        paths,
        stations,
        testPt,
        viz.zoom.getSnapDistance() / 2,
        visibleLayers
      );
    },
    selectJunction: function (testPt) {
      return hitTest.selectJunction(
        junctions,
        testPt,
        viz.zoom.getSnapDistance() / 2,
        visibleLayers
      );
    },
    hitTestRect: function (testPt1, testPt2) {
      return hitTest.rect(
        junctions,
        paths,
        stations,
        testPt1,
        testPt2,
        viz.zoom.getSnapDistance() / 2,
        visibleLayers
      );
    },
    hitTestLandmark: function (testL) {
      return hitTest.landmark(landmarks, testL);
    },
    hitTestFzPerimeter: function (fz, testZ) {
      return hitTestMap.regionPerimeter(fz.points, testZ, viz.zoom.getSnapDistance() / 2);
    },
    lineSegmentsIntersect: hitTestMap.lineSegmentsIntersect,
    lookupConnections: lookupConnections,
    lookupPathsFromJunctions: lookupPathsFromJunctions,
    lookupStationsFromJunction: lookupStationsFromJunction,
    lookupJunctionsFromObjects: lookupJunctionsFromObjects,

    setOcg: setOcg,

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

    addLandmark: addLandmark,
    updateLandmark: updateLandmark,
    removeLandmark: removeLandmark,

    addForbiddenZone: addForbiddenZone,
    updateForbiddenZone: updateForbiddenZone,
    removeForbiddenZone: removeForbiddenZone,

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

    setActiveLayer: setActiveLayer,
    setVisibleLayer: setVisibleLayer,

    ocg: () => ocgChoices[ocg],
    ocgId: () => ocg,
    rawOcgChoices: () => ocgChoices,
    rawJunctions: () => junctions,
    rawPaths: () => paths,
    rawStations: () => stations,
    rawLandmarks: () => landmarks,
    rawForbiddenZones: () => forbiddenZones,
    rawNoRotateZones: () => noRotateZones,
    rawLocHintZones: () => locHintZones,
    params: () => params,
    rawChangesets: () => changesets,
    rawSearch: () => search,
    rawActiveLayer: () => activeLayer,
    rawVisibleLayers: () => visibleLayers,
    isDirty: () => dirty
  };
}
