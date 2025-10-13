/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';
import SkillDesc from '../../../../routes/(config-panel)/config/task-templates/add/models/skill-desc';
import TaskTemplateMeta from '$lib/shared/task-template/models/task-template-meta.js';

export default function (viz) {
  var skillDescriptions = {};
  var taskTemplateMetas = {};
  var action;
  var callStack = [];
  var modelReady = false;
  var actionReady = false;

  // live-data
  var curTaskTemplateMeta;
  var curDepth = 0;

  function load(options) {
    if (options.skill_descriptions && Array.isArray(options.skill_descriptions)) {
      skillDescriptions = _.keyBy(
        _.map(options.skill_descriptions, (s) => new SkillDesc(s)),
        'id'
      );
    }
    if (options.task_templates && Array.isArray(options.task_templates)) {
      taskTemplateMetas = _.keyBy(
        _.map(options.task_templates, (t) => new TaskTemplateMeta(t)),
        'id'
      );
    }
    modelReady = true;
    triggerUpdate();
  }

  function update(data) {
    if (_.isEqual(action, data.action) && _.isEqual(callStack, data.callStack)) {
      return;
    }
    action = data.action;
    callStack = data.callStack;
    actionReady = true;
    triggerUpdate();
  }

  function triggerUpdate() {
    if (!modelReady || !actionReady) {
      return;
    }
    updateLiveData();
    viz.modelsUpdated();
  }

  function updateLiveData() {
    curTaskTemplateMeta = null;

    if (callStack.length < 2) {
      return;
    }

    let d = getDepth();
    d = d === 0 ? callStack.length - 1 : d;
    // get current task template
    let curAction = callStack[d];
    let curSkillId = callStack[d - 1];

    curTaskTemplateMeta = taskTemplateMetas[curSkillId[1]];
    if (!curTaskTemplateMeta) {
      console.log('LiveTaskTemplate unknown taskTemplateMeta');
      return;
    }

    viz.scene.activeObject.set(Number.parseInt(curAction[0]), null);
  }

  function getDepth() {
    if (!actionReady) {
      return 0;
    }
    let d = callStack.length - 1;
    d = d < 0 ? 0 : d;
    if (curDepth === 0) {
      return d;
    }
    return d > curDepth ? curDepth : d;
  }

  function goUp() {
    let d = getDepth();
    if (d === 0) {
      return;
    }
    d = d - 1;
    d = d < 1 ? 1 : d;
    curDepth = d;
    triggerUpdate();
  }

  function goDown() {
    let d = getDepth();
    if (d === 0) {
      return;
    }
    d = d + 1;
    if (callStack) {
      d = callStack.length - 1 < d ? callStack.length - 1 : d;
    }
    d = d < 0 ? 0 : d;
    curDepth = d;
    triggerUpdate();
  }

  function resetDepth() {
    curDepth = 0;
    triggerUpdate();
  }

  return {
    load: load,
    update: update,

    skillDescriptions: () => skillDescriptions,
    taskTemplateMetas: () => taskTemplateMetas,

    rawOutcomes: () => (curTaskTemplateMeta ? curTaskTemplateMeta.outcomes : []),
    rawActions: () => (curTaskTemplateMeta ? curTaskTemplateMeta.actions : []),
    rawTaskTemplateName: () => (curTaskTemplateMeta ? curTaskTemplateMeta.name : ''),

    isManualLayout: () => false,

    getDepth: getDepth,
    goUp: goUp,
    goDown: goDown,
    resetDepth: resetDepth
  };
}
