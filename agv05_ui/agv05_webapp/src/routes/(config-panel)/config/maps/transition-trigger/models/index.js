/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import * as _ from 'lodash-es';

import SkillDesc from 'task-template-editor/models/skill-desc';
import TaskTemplateMeta from '$lib/shared/models/task-template-meta.js';
import TransitionTrigger from './transition-trigger';

export default function (dynform) {
  /* Model for transition-trigger data */
  var transitionTriggers = [];
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
    if (options.transitionTrigger && Array.isArray(options.transitionTrigger)) {
      transitionTriggers = _.map(options.transitionTrigger, (t) => new TransitionTrigger(t));
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
  function getTransitionTrigger() {
    return _.map(transitionTriggers, (transitionTrigger) => transitionTrigger.__pick__());
  }

  /* Manipulation: TransitionTrigger */
  function addTransitionTrigger(transitionTrigger) {
    var idx = transitionTriggers.indexOf(transitionTrigger);
    if (idx >= 0) {
      return idx;
    }
    transitionTriggers.push(new TransitionTrigger(transitionTrigger));
    triggerDirty();
    return transitionTriggers.length - 1;
  }

  function cloneTransitionTrigger(transitionTrigger) {
    var idx = transitionTriggers.indexOf(transitionTrigger);
    if (idx < 0) {
      return;
    }
    transitionTriggers.splice(idx + 1, 0, new TransitionTrigger(transitionTrigger));
    triggerDirty();
    return idx + 1;
  }

  function updateTransitionTrigger(transitionTrigger) {
    var idx = transitionTriggers.indexOf(transitionTrigger);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function moveDownTransitionTrigger(transitionTrigger) {
    var idx;
    if (Number.isInteger(transitionTrigger)) {
      idx = transitionTrigger;
    } else {
      idx = transitionTriggers.indexOf(transitionTrigger);
    }
    if (idx < 0 || idx >= transitionTriggers.length - 1) {
      return;
    }

    var tmp = transitionTriggers[idx];
    transitionTriggers[idx] = transitionTriggers[idx + 1];
    transitionTriggers[idx + 1] = tmp;
    triggerDirty();
    return true;
  }

  function moveUpTransitionTrigger(transitionTrigger) {
    var idx;
    if (Number.isInteger(transitionTrigger)) {
      idx = transitionTrigger;
    } else {
      idx = transitionTriggers.indexOf(transitionTrigger);
    }
    if (idx < 1 || idx >= transitionTriggers.length) {
      return;
    }

    var tmp = transitionTriggers[idx];
    transitionTriggers[idx] = transitionTriggers[idx - 1];
    transitionTriggers[idx - 1] = tmp;
    triggerDirty();
    return true;
  }

  function removeTransitionTrigger(transitionTrigger) {
    var idx;
    if (Number.isInteger(transitionTrigger)) {
      idx = transitionTrigger;
    } else {
      idx = transitionTriggers.indexOf(transitionTrigger);
    }
    if (idx < 0 || idx >= transitionTriggers.length) {
      return;
    }

    transitionTriggers.splice(idx, 1);
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
    getTransitionTrigger: getTransitionTrigger,

    addTransitionTrigger: addTransitionTrigger,
    cloneTransitionTrigger: cloneTransitionTrigger,
    updateTransitionTrigger: updateTransitionTrigger,
    moveDownTransitionTrigger: moveDownTransitionTrigger,
    moveUpTransitionTrigger: moveUpTransitionTrigger,
    removeTransitionTrigger: removeTransitionTrigger,

    externalTriggerDirty: triggerDirty,

    rawTransitionTriggers: () => transitionTriggers,
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
