/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import base64 from 'base64-js';

import Station from '../models/station';
import Junction from '../models/junction';
import Path from '../models/path';
import Landmark from '../models/landmark';
import ForbiddenZone from '../models/forbidden-zone';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import liveAppChannel from 'stores/sock/live-app-channel';
import Pose2D from 'mapx-layout-editor/models/pose2d';
import lineExtraction from './line-extraction';

var agvChannel = liveAppChannel.getSub('trackless-agv');

const TIMEOUT = 10000;
const REFLECTORCOUNT = 7;
const REFLECTORDIST = 0.5 * 0.5;
const REFLECTORRADIUS = 0.2 * 0.2;
const WALLSAMPLE = 10;
const WALLRANGEOUTLIER = 0.2;
export default function (viz) {
  var $viz = $(viz.node());
  var enabled = false;
  var connected = false;
  var working = true;
  var intervalId;
  var lastMessage;
  var trackerStation;
  var trackerHeading;
  var trackerPos = {
    x: undefined,
    y: undefined
  };
  var robotPose;
  var poseValid = false;
  var laserPose;
  var laser2Pose;

  // Deadman prop
  var navToCmd;
  var navToCmdSuccessCB;
  var navToCmdActive = false;

  function enable() {
    if (enabled) {
      return;
    }
    agvChannel.subscribe(agvCallback);
    intervalId = setInterval(timer, 1000);
    enabled = true;
    setTrackerInvalid();
  }

  function disable() {
    if (!enabled) {
      return;
    }
    agvChannel.unsubscribe(agvCallback);
    clearInterval(intervalId);
    enabled = false;
    onDisconnected();
  }

  function agvCallback(data) {
    // Agv pose as heartbeat.
    lastMessage = Date.now();
    if (!connected) {
      onConnected();
    }
    if (data.id === 'robot_status') {
      if (working !== data.working) {
        if (data.working) {
          onWorking();
        } else {
          onIdle();
        }
      }
    } else if (data.id === 'robot_svg') {
      viz.scene.live.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'robot_pose') {
      robotPose = data.robot_pose;
      viz.scene.live.updateRobotPose(getCorrectedPose(robotPose));
    } else if (data.id === 'laser_pose') {
      updateLaserPose(data.laser_pose);
      viz.scene.live.updateLaserPose(data.laser_pose);
    } else if (data.id === 'laser_scan') {
      data.laser_scan.ranges = new Float32Array(
        base64.toByteArray(data.laser_scan.ranges).buffer
      );
      if (data.laser_scan.intensities) {
        data.laser_scan.intensities = new Float32Array(
          base64.toByteArray(data.laser_scan.intensities).buffer
        );
      }
      updateLaserScan(data.laser_scan);
      viz.scene.live.updateLaserScan(data.laser_scan);
    } else if (data.id === 'reflectors') {
      updateReflectors(data.reflectors);
      viz.scene.live.updateReflectors(data.reflectors);
    } else if (data.id === 'stop') {
      onDisconnected();
    } else if (data.id === 'action_completed') {
      if (data.uuid === navToCmd.uuid) {
        if (navToCmdSuccessCB) {
          navToCmdSuccessCB();
        }
        navToCmd = null;
        navToCmdSuccessCB = null;
        navToCmdActive = false;
      }
    }
  }

  function timer() {
    if (!connected || Date.now() - lastMessage < TIMEOUT) {
      return;
    }
    onDisconnected();
  }

  function onConnected() {
    connected = true;
    viz.scene.live.show();
    setTrackerInvalid();
    agvChannel.publish({
      command: 'retrieve_all'
    });
  }

  function onDisconnected() {
    connected = false;
    viz.scene.live.hide();
    setTrackerInvalid();
  }

  function onWorking() {
    working = true;
    $viz.trigger('live.updated');
  }

  function onIdle() {
    working = false;
    $viz.trigger('live.updated');
  }

  function modelsUpdated() {}

  // Position Tracker
  function setTrackerStation(station) {
    let junctions = viz.models.rawJunctions();
    let stations = viz.models.rawStations();
    if (!stations.includes(station)) {
      return;
    }
    let j = junctions[station.j];

    trackerStation = station;
    trackerHeading = station.heading;
    trackerPos.x = j.x;
    trackerPos.y = j.y;
  }

  function setTrackerInvalid(invalidPose = true) {
    trackerStation = null;
    trackerHeading = null;
    trackerPos.x = null;
    trackerPos.y = null;

    navToCmd = null;
    navToCmdSuccessCB = null;
    navToCmdActive = false;

    if (invalidPose) {
      poseValid = false;
    }
    $viz.trigger('live.updated');
  }

  function isTrackerValid() {
    let junctions = viz.models.rawJunctions();
    let stations = viz.models.rawStations();

    if (!trackerStation || !stations.includes(trackerStation)) {
      return false;
    }
    if (trackerStation.heading !== trackerHeading) {
      return false;
    }
    let j = junctions[trackerStation.j];
    return trackerPos.x === j.x && trackerPos.y === j.y;
  }

  function isPoseValid() {
    return !!robotPose && poseValid;
  }

  // Manipulation
  function canSetStation() {
    var ob = viz.scene.activeObject.get();
    return (
      connected &&
      !working &&
      ob instanceof Station &&
      viz.models.rawStations().indexOf(ob) >= 0 &&
      (ob.heading !== Station.Heading.NA || isPoseValid())
    );
  }

  function setStation() {
    var ob = viz.scene.activeObject.get();
    if (!canSetStation()) {
      return;
    }
    let stationIdx = viz.models.rawStations().indexOf(ob);

    var rawJunctions = viz.models.rawJunctions();
    var rawPaths = viz.models.rawPaths();
    var rawStations = viz.models.rawStations().map(function (s, i) {
      var station = new Station(s);
      station.name = `station_${i}`;
      if (stationIdx === i) {
        if (station.heading === Station.Heading.NA) {
          var robotHeading = Math.floor((robotPose.theta * 180.0) / Math.PI);
          robotHeading = robotHeading < 0 ? robotHeading + 360 : robotHeading;
          station.heading = robotHeading;
        }
      }
      return station;
    });
    var rawLandmarks = viz.models.rawLandmarks();
    var rawForbiddenZones = viz.models.rawForbiddenZones();
    var rawNoRotateZones = viz.models.rawNoRotateZones();
    var structure = getStructure(
      rawJunctions,
      rawPaths,
      rawStations,
      rawLandmarks,
      rawForbiddenZones,
      rawNoRotateZones
    );
    let metadata = JSON.stringify(viz.models.getMetadata());
    let stations = JSON.stringify(_.map(rawStations, 'name'));
    let ocg = viz.models.ocg();
    agvChannel.publish({
      command: 'set_station',
      metadata: metadata,
      structure: structure,
      stations: stations,
      station_idx: stationIdx,
      ocg: ocg.id
    });

    setTrackerStation(ob);
    poseValid = true;
    $viz.trigger('live.updated');
  }

  function resetTracker() {
    var ob = viz.scene.activeObject.get();
    if (!isPoseValid() || !canSetStation()) {
      return;
    }
    setTrackerStation(ob);
    $viz.trigger('live.updated');
  }

  function canNavTo() {
    var ob = viz.scene.activeObject.get();
    if (
      !connected ||
      !(ob instanceof Station) ||
      ob === trackerStation ||
      working ||
      !isTrackerValid() ||
      !isPoseValid()
    ) {
      return false;
    }
    return true;
  }

  function canGoTo() {
    let ob = viz.scene.activeObject.get();
    if (
      !connected ||
      !(ob instanceof Station) ||
      ob === trackerStation ||
      working ||
      !isPoseValid()
    ) {
      return false;
    }
    return true;
  }

  function setNavTo(reverse = false) {
    var ob = viz.scene.activeObject.get();
    if (!canNavTo()) {
      return;
    }

    var rawJunctions = viz.models.rawJunctions();
    var rawPaths = viz.models.rawPaths();
    var rawStations = viz.models.rawStations().map(function (s, i) {
      var station = new Station(s);
      station.name = `station_${i}`;
      return station;
    });
    var endStation = rawStations[viz.models.rawStations().indexOf(ob)];
    var ocg = viz.models.ocg();
    var rawMetadata = viz.models.getMetadata();
    var rawLandmarks = viz.models.rawLandmarks();
    var rawForbiddenZones = viz.models.rawForbiddenZones();
    var rawNoRotateZones = viz.models.rawNoRotateZones();
    setNavToCmd(
      ocg.id,
      rawMetadata,
      rawJunctions,
      rawPaths,
      rawStations,
      rawLandmarks,
      rawForbiddenZones,
      rawNoRotateZones,
      trackerStation,
      endStation,
      reverse,
      () => {
        setTrackerStation(ob);
        $viz.trigger('live.updated');
      }
    );
  }

  function setGoTo(reverse = false, dynamic = false) {
    var ob = viz.scene.activeObject.get();
    if (!canGoTo()) {
      return;
    }
    if (dynamic && !viz.options.dynamic) {
      return;
    }

    var j = viz.models.rawJunctions()[ob.j];
    var robotHeading = Math.floor((robotPose.theta * 180.0) / Math.PI);
    robotHeading = robotHeading < 0 ? robotHeading + 360 : robotHeading;
    var rawJunctions = [
      new Junction({
        x: robotPose.x,
        y: robotPose.y
      }),
      new Junction(j)
    ];
    var rawPaths = [
      new Path({
        j1: 0,
        j2: 1,
        distance: 1,
        dynamic: dynamic
      })
    ];
    var rawStations = [
      new Station({
        name: 'station_0',
        j: 0,
        heading: robotHeading
      }),
      new Station({
        name: 'station_1',
        j: 1,
        heading: ob.heading
      })
    ];
    var ocg = viz.models.ocg();
    var rawMetadata = viz.models.getMetadata();
    var rawLandmarks = viz.models.rawLandmarks();
    var rawForbiddenZones = viz.models.rawForbiddenZones();
    var rawNoRotateZones = viz.models.rawNoRotateZones();
    setNavToCmd(
      ocg.id,
      rawMetadata,
      rawJunctions,
      rawPaths,
      rawStations,
      rawLandmarks,
      rawForbiddenZones,
      rawNoRotateZones,
      rawStations[0],
      rawStations[1],
      reverse,
      () => {
        setTrackerStation(ob);
        $viz.trigger('live.updated');
      }
    );
  }

  function canSetPose() {
    return connected && !working;
  }

  function setPose(pose, ocgId) {
    if (!canSetPose()) {
      return;
    }

    var metadata = JSON.stringify({});
    var structure = JSON.stringify({
      junction_count: 1,
      junctions: [
        {
          x: pose.x,
          y: pose.y
        }
      ],
      paths: [],
      stations: [],
      landmarks: [],
      forbidden_zones: [],
      no_rotate_zones: []
    });
    var stations = JSON.stringify([]);

    agvChannel.publish({
      command: 'set_pose',
      ocg: ocgId,
      pose: pose,
      metadata: metadata,
      structure: structure,
      stations: stations
    });
    poseValid = true;
  }

  function canGoToPose() {
    if (!connected || working || !isPoseValid()) {
      return false;
    }
    return true;
  }

  function setGoToPose(pose, ocgId, reverse = false, dynamic = false) {
    if (!canGoToPose()) {
      return;
    }
    if (dynamic && !viz.options.dynamic) {
      return;
    }

    var robotHeading = Math.floor((robotPose.theta * 180.0) / Math.PI);
    robotHeading = robotHeading < 0 ? robotHeading + 360 : robotHeading;
    var stationHeading = Math.floor((pose.theta * 180.0) / Math.PI);
    stationHeading = stationHeading < 0 ? stationHeading + 360 : stationHeading;
    var rawJunctions = [
      new Junction({
        x: robotPose.x,
        y: robotPose.y
      }),
      new Junction({
        x: pose.x,
        y: pose.y
      })
    ];
    var rawPaths = [
      new Path({
        j1: 0,
        j2: 1,
        distance: 1,
        dynamic: dynamic
      })
    ];
    var rawStations = [
      new Station({
        name: 'station_0',
        j: 0,
        heading: robotHeading
      }),
      new Station({
        name: 'station_1',
        j: 1,
        heading: stationHeading
      })
    ];
    var rawMetadata = {};
    var rawLandmarks = [];
    var rawForbiddenZones = [];
    var rawNoRotateZones = [];
    setNavToCmd(
      ocgId,
      rawMetadata,
      rawJunctions,
      rawPaths,
      rawStations,
      rawLandmarks,
      rawForbiddenZones,
      rawNoRotateZones,
      rawStations[0],
      rawStations[1],
      reverse
    );
  }

  function setNavToCmd(
    ocgId,
    rawMetadata,
    rawJunctions,
    rawPaths,
    rawStations,
    rawLandmarks,
    rawForbiddenZones,
    rawNoRotateZones,
    startStation,
    endStation,
    reverse,
    onSuccessCB
  ) {
    var location = [startStation.j, startStation.heading];
    var structure = getStructure(
      rawJunctions,
      rawPaths,
      rawStations,
      rawLandmarks,
      rawForbiddenZones,
      rawNoRotateZones
    );
    let metadata = JSON.stringify(rawMetadata);
    let stations = JSON.stringify(_.map(rawStations, 'name'));

    if (location[1] === 360) {
      var robotHeading = Math.floor((robotPose.theta * 180.0) / Math.PI);
      robotHeading = robotHeading < 0 ? robotHeading + 360 : robotHeading;
      location[1] = robotHeading;
    }

    let params = {
      station: endStation.name
    };

    navToCmd = {
      command: 'nav_to',
      uuid: generateUUID(),
      reverse: reverse,
      metadata: metadata,
      structure: structure,
      stations: stations,
      location: location,
      params: params,
      ocg: ocgId
    };
    navToCmdActive = false;
    navToCmdSuccessCB = onSuccessCB;
    $viz.trigger('live.updated');
  }

  function generateUUID() {
    const array = new Uint8Array(16);
    window.crypto.getRandomValues(array);
    // Set specific bits to signify the version and variant
    array[6] = (array[6] & 0x0f) | 0x40; // version 4
    array[8] = (array[8] & 0x3f) | 0x80; // variant 10xx
    const uuid = Array.from(array)
      .map((b) => ('0' + b.toString(16)).slice(-2))
      .join('');

    return `${uuid.substr(0, 8)}-${uuid.substr(8, 4)}-${uuid.substr(12, 4)}-${uuid.substr(16, 4)}-${uuid.substr(20, 12)}`;
  }

  function canSendNavToCmd() {
    if (!connected || !navToCmd) {
      return false;
    }
    return true;
  }

  function sendNavToCmd() {
    if (!canSendNavToCmd()) {
      return;
    }

    // clear tracker as we are moving.
    trackerStation = null;
    trackerHeading = null;
    trackerPos.x = null;
    trackerPos.y = null;

    if (!navToCmdActive) {
      agvChannel.publish(navToCmd);
      navToCmdActive = true;
      return;
    }

    agvChannel.publish({
      command: 'heartbeat',
      uuid: navToCmd.uuid
    });
  }

  function canAbortAction() {
    if (!connected || !working) {
      return false;
    }
    return true;
  }

  function abortAction() {
    if (!canAbortAction()) {
      return;
    }

    agvChannel.publish({
      command: 'abort'
    });
    setTrackerInvalid(false);
  }

  function getStructure(junctions, paths, stations, landmarks, forbiddenZones, noRotateZones) {
    return JSON.stringify({
      junction_count: junctions.length,
      junctions: _.map(junctions, (junction) => _.pick(junction, Junction.__attrs__)),
      paths: _.map(paths, (path) => _.pick(path, Path.__attrs__)),
      stations: _.map(stations, (station) => _.pick(station, Station.__attrs__)),
      landmarks: _.map(landmarks, (landmark) => _.pick(landmark, Landmark.__attrs__)),
      forbidden_zones: _.map(forbiddenZones, (zone) => _.pick(zone, ForbiddenZone.__attrs__)),
      no_rotate_zones: _.map(noRotateZones, (zone) => _.pick(zone, NoRotateZone.__attrs__))
    });
  }

  // Manual set pose
  var poseCorrection = false;
  var correction;

  function startPoseCorrection() {
    poseCorrection = true;
    correction = new Pose2D();
  }

  function updatePoseCorrection(action, scale) {
    let newPose = new Pose2D(robotPose).add(correction);
    if (action === 'Up') {
      newPose.x += 0.01 * scale;
    } else if (action === 'Down') {
      newPose.x -= 0.01 * scale;
    } else if (action === 'Left') {
      newPose.y += 0.01 * scale;
    } else if (action === 'Right') {
      newPose.y -= 0.01 * scale;
    } else if (action === 'RL') {
      newPose.theta += ((0.5 * Math.PI) / 180.0) * scale;
    } else if (action === 'RR') {
      newPose.theta -= ((0.5 * Math.PI) / 180.0) * scale;
    }
    correction = newPose.sub2(robotPose);
  }

  function applyPoseCorrection(data) {
    poseCorrection = false;
    if (data === true) {
      let newPose = new Pose2D(robotPose).add(correction);
      setPose(newPose, viz.models.ocgId());
    }
  }

  function getCorrectedPose(data) {
    if (!poseCorrection) {
      return data;
    }
    return new Pose2D(data).add(correction);
  }

  // Apply reflectors
  var reflector;
  var reflectorData;
  var reflectorTimeout;

  function canApplyReflector() {
    return connected && !working && isPoseValid();
  }

  function applyReflector(callback) {
    reflector = callback;
    reflectorData = {};
    reflectorTimeout = setTimeout(() => {
      if (!reflector) {
        return;
      }
      reflector = undefined;
      callback('Timeout fail to obtain reflector data.');
    }, 10000);
  }

  function _endReflector(error) {
    if (!reflector) {
      return;
    }
    if (reflectorTimeout) {
      clearTimeout(reflectorTimeout);
      reflectorTimeout = undefined;
    }
    reflector(error);
    reflector = undefined;
  }

  function updateLaserPose(data) {
    if ('laser' in data) {
      laserPose = data.laser;
    }
    if ('laser2' in data) {
      laser2Pose = data.laser2;
    }
  }

  function updateReflectors(data) {
    if (!reflector || !isPoseValid()) {
      return;
    }
    if (['laser', 'laser2'].indexOf(data.frame_id) < 0) {
      return;
    }
    let pose = data.frame_id === 'laser' ? laserPose : laser2Pose;
    if (!pose) {
      return;
    }
    if (!(data.frame_id in reflectorData)) {
      reflectorData[data.frame_id] = [];
    }
    let d = reflectorData[data.frame_id];

    if (d.length > 0) {
      if (d[0].length !== data.points.length) {
        _endReflector('Reflector detection unstable.');
        return;
      }
    }
    let points = data.points.map((point) => {
      let pointInRobot = transformPoint(point, pose);
      let pointInWorld = transformPoint(pointInRobot, robotPose);
      return pointInWorld;
    });
    d.push(points);

    validateAndApplyReflectors();
  }

  function validateAndApplyReflectors() {
    let rData = Object.values(reflectorData);
    if (!rData.every((l) => l.length >= REFLECTORCOUNT)) {
      return;
    }
    // Calculate final reflector location and apply.
    let invalid = false;
    let reflectorsCollected = [];
    for (let i = 0; i < rData.length; i++) {
      let rd = rData[i];
      let sum;
      let first;
      for (let j = 0; j < rd.length; j++) {
        let points = rd[j];
        if (!sum) {
          sum = points;
          first = points;
          continue;
        }
        for (let k = 0; k < first.length; k++) {
          let p1 = first[k];
          let p2 = points[k];
          if (sqDist(p1, p2) > REFLECTORDIST) {
            invalid = true;
            break;
          }
        }
        if (invalid) {
          break;
        }
        sum = sum.map((point, k) => {
          let p = points[k];
          return [point[0] + p[0], point[1] + p[1]];
        });
      }
      if (invalid) {
        break;
      }
      sum = sum.map((point) => [point[0] / rd.length, point[1] / rd.length]);
      reflectorsCollected = reflectorsCollected.concat(sum);
    }
    if (invalid) {
      _endReflector('Reflector data collected unstable.');
      return;
    }

    if (reflectorsCollected.length <= 0) {
      return;
    }

    // Hit test between collected reflectors.
    let raw = reflectorsCollected;
    reflectorsCollected = [raw[0]];
    for (let i = 1; i < raw.length; i++) {
      const p1 = raw[i];
      let matchP = reflectorsCollected.find((p2) => {
        return sqDist(p1, p2) <= REFLECTORRADIUS;
      });
      if (matchP) {
        matchP[0] = (matchP[0] + p1[0]) / 2;
        matchP[1] = (matchP[1] + p1[1]) / 2;
      } else {
        reflectorsCollected.push(p1);
      }
    }

    // Apply changes.
    viz.models.push('liveAppApplyLandmark');
    reflectorsCollected.forEach((pt) => {
      let lm = new Landmark({
        x: pt[0],
        y: pt[1]
      });
      let hit = viz.models.hitTestLandmark(lm);
      if (!hit) {
        viz.models.addLandmark(lm, false);
      } else {
        hit.x = lm.x;
        hit.y = lm.y;
        viz.models.updateLandmark(hit);
      }
    });
    _endReflector();
  }

  function sqDist(a, b) {
    var d0 = a[0] - b[0];
    var d1 = a[1] - b[1];
    return d0 * d0 + d1 * d1;
  }

  function transformPoint([x, y], pose) {
    let cosTheta = Math.cos(pose.theta);
    let sinTheta = Math.sin(pose.theta);
    let xNew = cosTheta * x - sinTheta * y + pose.x;
    let yNew = sinTheta * x + cosTheta * y + pose.y;
    if (pose.flip) {
      yNew = -yNew;
    }
    return [xNew, yNew];
  }

  // Apply wall
  var wall;
  var wallData;
  var wallTimeout;

  function canApplyWall() {
    return connected && !working && isPoseValid();
  }

  function applyWall(callback) {
    wall = callback;
    wallData = {};
    wallTimeout = setTimeout(() => {
      if (!wall) {
        return;
      }
      wall = undefined;
      callback('Timeout fail to obtain laser data.');
    }, 10000);
  }

  function _endWall(error) {
    if (!wall) {
      return;
    }
    if (wallTimeout) {
      clearTimeout(wallTimeout);
      wallTimeout = undefined;
    }
    wall(error);
    wall = undefined;
  }

  function updateLaserScan(data) {
    if (!wall || !isPoseValid()) {
      return;
    }
    if (['laser', 'laser2'].indexOf(data.frame_id) < 0) {
      return;
    }
    let pose = data.frame_id === 'laser' ? laserPose : laser2Pose;
    if (!pose) {
      return;
    }
    if (!(data.frame_id in wallData)) {
      wallData[data.frame_id] = {
        rangesData: [],
        frame_id: data.frame_id,
        angle_min: data.angle_min,
        angle_max: data.angle_max,
        angle_increment: data.angle_increment
      };
    }
    let d = wallData[data.frame_id];
    d.rangesData.push(data.ranges);

    validateAndApplyWall();
  }

  function validateAndApplyWall() {
    let wData = Object.values(wallData);
    if (!wData.every((wd) => wd.rangesData.length >= WALLSAMPLE)) {
      return;
    }
    let wallCollected = [];
    for (let i = 0; i < wData.length; i++) {
      let wd = wData[i];
      let sCount = wd.rangesData.length;
      let firstRanges = wd.rangesData[0];
      let rangeCount = firstRanges.length;
      let averageRanges = new Float32Array(rangeCount);
      let counts = new Uint8Array(rangeCount);
      for (let j = 0; j < sCount; j++) {
        let ranges = wd.rangesData[j];
        for (let k = 0; k < rangeCount; k++) {
          if (Math.abs(ranges[k] - firstRanges[k]) > WALLRANGEOUTLIER) {
            continue;
          }
          counts[k] += 1;
          averageRanges[k] += ranges[k];
        }
      }
      for (let k = 0; k < rangeCount; k++) {
        averageRanges[k] /= counts[k];
      }

      let pose = wd.frame_id === 'laser' ? laserPose : laser2Pose;
      if (!pose) {
        continue;
      }
      let transfrom = (point) => {
        let pointInRobot = transformPoint(point, pose);
        let pointInWorld = transformPoint(pointInRobot, robotPose);
        return pointInWorld;
      };
      let lines = lineExtraction(
        averageRanges,
        wd.angle_min,
        wd.angle_max,
        wd.angle_increment
      );
      lines.forEach((line) => {
        let start = [line.start.x, line.start.y];
        let end = [line.end.x, line.end.y];
        line.start = transfrom(start);
        line.end = transfrom(end);
      });
      wallCollected = wallCollected.concat(lines);
    }

    if (!viz.canvas) {
      _endWall('Missing canvas.');
      return;
    }
    let canvas = viz.canvas;

    wallCollected.forEach((line) => {
      let c1 = canvas.ptFromDiv(viz.zoom.coordToPixel(line.start));
      let c2 = canvas.ptFromDiv(viz.zoom.coordToPixel(line.end));
      let args = c1.concat(c2);
      canvas.drawLine.apply(null, args);
    });
    if (wallCollected.length > 0) {
      viz.models.updatePng(canvas.node());
    }
    _endWall();
  }

  function destroy() {
    _endReflector();
    _endWall();
    disable();
  }

  return {
    enabled: () => enabled,
    connected: () => connected,
    working: () => working,

    enable: enable,
    disable: disable,

    modelsUpdated: modelsUpdated,
    canSetStation: canSetStation,
    setStation: setStation,
    resetTracker: resetTracker,
    canNavTo: canNavTo,
    canGoTo: canGoTo,
    setNavTo: setNavTo,
    setGoTo: setGoTo,
    canSetPose: canSetPose,
    setPose: setPose,
    canGoToPose: canGoToPose,
    setGoToPose: setGoToPose,
    canAbortAction: canAbortAction,
    abortAction: abortAction,

    canSendNavToCmd: canSendNavToCmd,
    sendNavToCmd: sendNavToCmd,

    isTrackerValid: isTrackerValid,
    isPoseValid: isPoseValid,
    setTrackerInvalid: setTrackerInvalid,

    rawRobotPose: () => robotPose,

    startPoseCorrection: startPoseCorrection,
    updatePoseCorrection: updatePoseCorrection,
    applyPoseCorrection: applyPoseCorrection,

    canApplyReflector: canApplyReflector,
    applyReflector: applyReflector,

    canApplyWall: canApplyWall,
    applyWall: applyWall,

    destroy: destroy
  };
}
