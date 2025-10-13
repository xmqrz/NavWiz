/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

import SkillDesc from 'task-template-editor/models/skill-desc';
import TaskTemplateMeta from '$lib/shared/models/task-template-meta.js';
import Teleport from './teleport';

export default function (dynform) {
  /* Model for teleport data */
  var teleports = [];
  var skillDescriptions = {};
  var taskTemplateMetas = {};
  var stationList = [];
  var registerList = [];
  var globalParams = [];
  var variables = [];
  var filter;
  var dirty = false;

  /* Deserialize from database */
  function load(options) {
    if (options.teleport && Array.isArray(options.teleport)) {
      teleports = _.map(options.teleport, (t) => new Teleport(t));
    }
    if (options.skillDescriptions && Array.isArray(options.skillDescriptions)) {
      skillDescriptions = _.keyBy(
        _.map(options.skillDescriptions, (s) => new SkillDesc(s)),
        'id'
      );
    }
    if (options.taskTemplateMetas && Array.isArray(options.taskTemplateMetas)) {
      taskTemplateMetas = _.keyBy(
        _.map(options.taskTemplateMetas, (t) => new TaskTemplateMeta(t)),
        'id'
      );
    }
    if (options.stationList && Array.isArray(options.stationList)) {
      stationList = options.stationList;
    }
    if (options.registerList && Array.isArray(options.registerList)) {
      registerList = options.registerList;
    }
    globalParams = [];
    if (options.reservedGlobalParams && Array.isArray(options.reservedGlobalParams)) {
      globalParams = globalParams.concat(options.reservedGlobalParams);
    }
    if (options.globalParams && Array.isArray(options.globalParams)) {
      globalParams = globalParams.concat(options.globalParams);
    }
    if (options.variables && Array.isArray(options.variables)) {
      variables = options.variables;
    }
    if (options.filter) {
      filter = options.filter;
    }
  }

  /* Serialize to database */
  function getTeleport() {
    return _.map(teleports, (teleport) => teleport.__pick__());
  }

  /* Manipulation: Teleport */
  function addTeleport(teleport) {
    var idx = teleports.indexOf(teleport);
    if (idx >= 0) {
      return idx;
    }
    teleports.push(new Teleport(teleport));
    triggerDirty();
    return teleports.length - 1;
  }

  function cloneTeleport(teleport) {
    var idx = teleports.indexOf(teleport);
    if (idx < 0) {
      return;
    }
    teleports.splice(idx + 1, 0, new Teleport(teleport));
    triggerDirty();
    return idx + 1;
  }

  function updateTeleport(teleport) {
    var idx = teleports.indexOf(teleport);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function moveDownTeleport(teleport) {
    var idx;
    if (Number.isInteger(teleport)) {
      idx = teleport;
    } else {
      idx = teleports.indexOf(teleport);
    }
    if (idx < 0 || idx >= teleports.length - 1) {
      return;
    }

    var tmp = teleports[idx];
    teleports[idx] = teleports[idx + 1];
    teleports[idx + 1] = tmp;
    triggerDirty();
    return true;
  }

  function moveUpTeleport(teleport) {
    var idx;
    if (Number.isInteger(teleport)) {
      idx = teleport;
    } else {
      idx = teleports.indexOf(teleport);
    }
    if (idx < 1 || idx >= teleports.length) {
      return;
    }

    var tmp = teleports[idx];
    teleports[idx] = teleports[idx - 1];
    teleports[idx - 1] = tmp;
    triggerDirty();
    return true;
  }

  function removeTeleport(teleport) {
    var idx;
    if (Number.isInteger(teleport)) {
      idx = teleport;
    } else {
      idx = teleports.indexOf(teleport);
    }
    if (idx < 0 || idx >= teleports.length) {
      return;
    }

    teleports.splice(idx, 1);
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
    getTeleport: getTeleport,

    addTeleport: addTeleport,
    cloneTeleport: cloneTeleport,
    updateTeleport: updateTeleport,
    moveDownTeleport: moveDownTeleport,
    moveUpTeleport: moveUpTeleport,
    removeTeleport: removeTeleport,

    externalTriggerDirty: triggerDirty,

    rawTeleports: () => teleports,
    skillDescriptions: () => skillDescriptions,
    taskTemplateMetas: () => taskTemplateMetas,
    stationList: () => stationList,
    registerList: () => registerList,
    globalParams: () => globalParams,
    variables: () => variables,
    rawFilter: () => filter,
    isDirty: () => dirty
  };
}
