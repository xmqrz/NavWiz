/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

import GlobalParam from '$lib/shared/models/param.js';

export default function (dynform) {
  /* Model for global_param data */
  var reservedGlobalParams = [];
  var globalParams = [];
  var stationList = [];
  var registerList = [];
  var filter;
  var dirty = false;

  /* Deserialize from database */
  function load(options) {
    if (options.reservedGlobalParams && Array.isArray(options.reservedGlobalParams)) {
      reservedGlobalParams = _.map(options.reservedGlobalParams, (gp) => new GlobalParam(gp));
    }
    if (options.globalParams && Array.isArray(options.globalParams)) {
      globalParams = _.map(options.globalParams, (gp) => new GlobalParam(gp));
    }
    if (options.stationList && Array.isArray(options.stationList)) {
      stationList = options.stationList;
    }
    if (options.registerList && Array.isArray(options.registerList)) {
      registerList = options.registerList;
    }
    if (options.filter) {
      filter = options.filter;
    }
  }

  /* Serialize to database */
  function getGlobalParams() {
    return _.map(globalParams, (gp) => _.pick(gp, GlobalParam.__attrs__));
  }

  /* Manipulation: GlobalParam */
  function addGlobalParam(gp) {
    var idx = globalParams.indexOf(gp);
    if (idx >= 0) {
      return idx;
    }
    globalParams.push(new GlobalParam(gp));
    triggerDirty();
    return globalParams.length - 1;
  }

  function updateGlobalParam(gp) {
    var idx = globalParams.indexOf(gp);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function moveDownGlobalParam(gp) {
    var idx;
    if (Number.isInteger(gp)) {
      idx = gp;
    } else {
      idx = globalParams.indexOf(gp);
    }
    if (idx < 0 || idx >= globalParams.length - 1) {
      return;
    }

    var tmp = globalParams[idx];
    globalParams[idx] = globalParams[idx + 1];
    globalParams[idx + 1] = tmp;
    triggerDirty();
    return true;
  }

  function moveUpGlobalParam(gp) {
    var idx;
    if (Number.isInteger(gp)) {
      idx = gp;
    } else {
      idx = globalParams.indexOf(gp);
    }
    if (idx < 1 || idx >= globalParams.length) {
      return;
    }

    var tmp = globalParams[idx];
    globalParams[idx] = globalParams[idx - 1];
    globalParams[idx - 1] = tmp;
    triggerDirty();
    return true;
  }

  function removeGlobalParam(gp) {
    var idx;
    if (Number.isInteger(gp)) {
      idx = gp;
    } else {
      idx = globalParams.indexOf(gp);
    }
    if (idx < 0 || idx >= globalParams.length) {
      return;
    }

    globalParams.splice(idx, 1);
    triggerDirty();
    return true;
  }

  function triggerDirty() {
    if (!dirty) {
      dirty = true;
      dynform.trigger('models.dirty');
    }
  }

  dynform.models = {
    load: load,
    getGlobalParams: getGlobalParams,

    addGlobalParam: addGlobalParam,
    updateGlobalParam: updateGlobalParam,
    moveDownGlobalParam: moveDownGlobalParam,
    moveUpGlobalParam: moveUpGlobalParam,
    removeGlobalParam: removeGlobalParam,

    externalTriggerDirty: triggerDirty,

    rawGlobalParams: () => globalParams,
    reservedGlobalParams: () => reservedGlobalParams,
    stationList: () => stationList,
    registerList: () => registerList,
    rawFilter: () => filter,
    isDirty: () => dirty
  };
}
