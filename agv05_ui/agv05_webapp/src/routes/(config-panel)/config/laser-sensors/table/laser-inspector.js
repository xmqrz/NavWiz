/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import watchdogF from './watchdog';

var panelHtml = `
<div class="flex space-x-3 items-center">
  <label class="label font-semibold">Obstacle Detected: </label>
  <span id="table-obstacle-on" class="hidden">
    <i class="fa fa-toggle-on text-xl"></i>
  </span>
  <span id="table-obstacle-off">
    <i class="fa fa-toggle-off text-xl"></i>
  </span>
</div>
`;

export default function (viz, container) {
  var panel = $(panelHtml);
  var obstacleOn = panel.find('#table-obstacle-on');
  var obstacleOff = panel.find('#table-obstacle-off');
  container.append(panel);

  var inspector = {};
  var watchdogs = {};
  var curScanId = [];
  var curMinActivation = 4;
  var region, minX, minY, maxX, maxY;

  function updateLaserPose(data) {
    for (let frameId in data) {
      if (!(frameId in watchdogs)) {
        let wd = watchdogF(inspector, frameId);
        watchdogs[frameId] = wd;
      }
      watchdogs[frameId].updateLaserPose(data);
    }
  }

  function updateLaserScan(data) {
    if (!curScanId.includes(data.scan_id) || !region || region.length < 3) {
      return;
    }

    let detected = false;
    for (let frameId in watchdogs) {
      let wd = watchdogs[frameId];
      wd.updateLaserScan(data);
      detected = detected || wd.getDetected();
    }

    obstacleOn.toggleClass('hidden', !detected);
    obstacleOff.toggleClass('hidden', detected);
  }

  function regionUpdated() {
    curScanId = viz.models.rawScanId();
    curMinActivation = viz.models.getMinActivation();

    for (let frameId in watchdogs) {
      watchdogs[frameId].reset();
    }

    let regions = viz.models.rawRegions();
    let newRegion = regions[viz.table.getActiveRegion()];
    updateRegion(newRegion);
  }

  function updateRegion(newRegion) {
    if (!newRegion) {
      return;
    }
    region = newRegion;

    minX = region[0][0];
    maxX = region[0][0];
    minY = region[0][1];
    maxY = region[0][1];

    for (let r of region) {
      minX = Math.min(minX, r[0]);
      maxX = Math.max(maxX, r[0]);
      minY = Math.min(minY, r[1]);
      maxY = Math.max(maxY, r[1]);
    }
  }

  inspector = {
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    regionUpdated: regionUpdated,
    getRegion: () => region,
    getRegionParameter: () => [minX, minY, maxX, maxY],
    getMinActivation: () => curMinActivation
  };

  return inspector;
}
