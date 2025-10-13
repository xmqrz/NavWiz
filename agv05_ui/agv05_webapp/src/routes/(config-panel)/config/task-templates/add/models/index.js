/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import Action from './action.js';
import Param from '$lib/shared/models/param.js';
import SkillDesc from './skill-desc.js';
import TaskTemplateMeta from '$lib/shared/models/task-template-meta.js';
import Variable from '$lib/shared/models/variable.js';

const MAX_UNDO_LEVELS = 150;

export default function (viz) {
  /* Model for map data */
  var $viz = $(viz.node());
  var id;
  var topLvl = true;
  var createSuspended = false;
  var params = [];
  var outcomes = ['End']; // preset to prevent initial triggerDirty() when adding new task template.
  var actions = [];
  var skillDescriptions = {};
  var taskTemplateMetas = {};
  var stationList = [];
  var registerList = [];
  var globalParams = [];
  var variables = [];
  var graph = [];
  var clipboard = null;
  var undoBuffer = [];
  var redoBuffer = [];
  var manualLayout = false;
  var corrupted = false;
  var dirty = false;
  var overwrite = false;
  var cached = false;
  var search;

  /* Deserialize from database */
  function load(options) {
    id = options.id;
    cached = !!options.cached;
    topLvl = !!options.topLvl;
    if (options.structure) {
      if (options.structure.actions && Array.isArray(options.structure.actions)) {
        let first = true;
        actions = _.map(options.structure.actions, function (action) {
          if (first) {
            first = false;
            if (Number.isInteger(action)) {
              return action;
            } else {
              return null;
            }
          }
          return new Action(action);
        });
      }
      manualLayout = !!options.structure.manual_layout;
    }
    if (actions.length === 0) {
      actions = [null];
    }

    if (options.metadata) {
      createSuspended = !!options.metadata.create_suspended;
      if (options.metadata.params && Array.isArray(options.metadata.params)) {
        params = _.map(options.metadata.params, (p) => new Param(p));
      }
      if (options.metadata.outcomes && Array.isArray(options.metadata.outcomes)) {
        outcomes = _.filter(
          _.map(options.metadata.outcomes, (outcome) => outcome.trim()),
          (outcome) => !!outcome
        );
      }
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
      globalParams = globalParams.concat(
        _.map(options.reservedGlobalParams, (p) => new Param(p))
      );
    }
    if (options.globalParams && Array.isArray(options.globalParams)) {
      globalParams = globalParams.concat(_.map(options.globalParams, (p) => new Param(p)));
    }
    if (options.variables && Array.isArray(options.variables)) {
      variables = _.map(options.variables, (v) => new Variable(v));
    }

    /* Sanitize actions */
    if (!viz.options.preserve) {
      for (let idx = 1; idx < actions.length; ) {
        let action = actions[idx];
        let skill;
        if (action && action.skillId) {
          if (action.skillId.indexOf('_ttpk_') === 0) {
            skill = taskTemplateMetas[action.skillId];
          } else {
            skill = skillDescriptions[action.skillId];
          }
        }
        if (!skill || !skill.name || !skill.outcomes) {
          removeAction(idx, true);
          corrupted = true;
          continue;
        }

        let param_keys = _.map(skill.params, 'key');
        for (let key in action.params) {
          if (param_keys.indexOf(key) < 0) {
            delete action.params[key];
            corrupted = true;
          }
        }

        for (let key in action.outcomes) {
          if (skill.outcomes.indexOf(key) < 0) {
            delete action.outcomes[key];
            corrupted = true;
          }
        }

        idx++;
      }
    }

    if (manualLayout) {
      for (let action of actions) {
        viz.scene.updateRenderMetadata(action);
      }
    }

    if (options.search) {
      search = options.search;
    }

    viz.modelsUpdated();
    // window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  /* Serialize to database */
  function getMetadata() {
    return {
      create_suspended: createSuspended,
      params: _.map(params, (param) => _.pick(param, Param.__metaAttrs__)),
      outcomes: outcomes
    };
  }

  function getStructure() {
    var first = true;
    return {
      manual_layout: manualLayout,
      action_count: actions.length,
      actions: _.map(actions, function (action) {
        if (first) {
          first = false;
          if (Number.isInteger(action)) {
            return action;
          } else {
            return null;
          }
        }
        return action.__pick__();
      })
    };
  }

  /* Manipulation: Is Top Level */
  function setTopLvl(s) {
    s = !!s;
    if (topLvl === s) {
      return;
    }
    if (s) {
      outcomes = ['End'];
    }
    topLvl = s;
    triggerDirty();
  }

  /* Manipulation: Create Suspended */
  function setCreateSuspended(s) {
    s = !!s;
    if (createSuspended === s) {
      return;
    }
    createSuspended = s;
    triggerDirty();
  }

  /* Manipulation: Param */
  function addParam(param) {
    var idx = params.indexOf(param);
    if (idx >= 0) {
      return idx;
    }
    params.push(new Param(param));
    triggerDirty();
    return params.length - 1;
  }

  function updateParam(param, updateAttrs) {
    var idx;
    if (Number.isInteger(param)) {
      idx = param;
    } else {
      idx = params.indexOf(param);
    }
    if (idx < 0 || idx >= params.length) {
      return;
    }

    param = params[idx];
    if (updateAttrs.name && updateAttrs.name !== param.name) {
      var oldRef = '${' + param.name + '}';
      var newRef = '${' + updateAttrs.name + '}';
      param.name = updateAttrs.name;

      // Update reference to param
      _updateParamReferences(oldRef, newRef);
    }
    if (updateAttrs.type && updateAttrs.type !== param.type) {
      var ref = '${' + param.name + '}';
      param.type = updateAttrs.type;

      // Remove reference to param (due to unmatched type)
      _updateParamReferences(ref, undefined);
    }
    if ('description' in updateAttrs) {
      param.description = updateAttrs.description;
    }
    if ('default' in updateAttrs) {
      param.default = updateAttrs.default;
    }
    if ('min' in updateAttrs) {
      param.min = updateAttrs.min;
    }
    if ('max' in updateAttrs) {
      param.max = updateAttrs.max;
    }
    triggerDirty();
    return idx;
  }

  function moveDownParam(param) {
    var idx;
    if (Number.isInteger(param)) {
      idx = param;
    } else {
      idx = params.indexOf(param);
    }
    if (idx < 0 || idx >= params.length - 1) {
      return;
    }

    var tmp = params[idx];
    params[idx] = params[idx + 1];
    params[idx + 1] = tmp;
    triggerDirty();
  }

  function moveUpParam(param) {
    var idx;
    if (Number.isInteger(param)) {
      idx = param;
    } else {
      idx = params.indexOf(param);
    }
    if (idx < 1 || idx >= params.length) {
      return;
    }

    var tmp = params[idx];
    params[idx] = params[idx - 1];
    params[idx - 1] = tmp;
    triggerDirty();
  }

  function removeParam(param) {
    var idx;
    if (Number.isInteger(param)) {
      idx = param;
    } else {
      idx = params.indexOf(param);
    }
    if (idx < 0 || idx >= params.length) {
      return;
    }

    var ref = '${' + params[idx].name + '}';
    params.splice(idx, 1);

    // Remove reference to param.
    _updateParamReferences(ref, undefined);
    triggerDirty();
  }

  function _updateParamReferences(oldRef, newRef) {
    var first = true;
    for (let action of actions) {
      if (first) {
        first = false;
        continue;
      }
      let updated = false;
      for (let k in action.params) {
        if (action.params[k] === oldRef) {
          updated = true;
          action.params[k] = newRef;
        }
      }

      if (manualLayout && updated) {
        viz.scene.updateRenderMetadata(action);
      }
    }
  }

  /* Manipulation: Outcomes */
  function updateOutcomes(o) {
    push('updateOutcomes');
    if (_.isString(o)) {
      o = o.split(',');
    }
    if (!Array.isArray(o)) {
      return;
    }
    o = _.filter(
      _.map(o, (oo) => oo.trim()),
      (oo) => !!oo
    );

    if (outcomes.join(',') === o.join(',')) {
      return;
    }
    if (o.length < outcomes.length) {
      // Remove reference to outcomes.
      var minOutcome = -o.length;
      var first = true;
      for (let action of actions) {
        if (first) {
          if (action < minOutcome) {
            actions[0] = null;
          }
          first = false;
          continue;
        }
        for (let k in action.outcomes) {
          if (action.outcomes[k] < minOutcome) {
            action.outcomes[k] = null;
          }
        }
      }
    }
    outcomes = o;
    triggerDirty();
  }

  /* Manipulation: Action */
  function addAction(action) {
    push('addAction');
    var idx = actions.indexOf(action);
    if (idx >= 0) {
      return idx;
    }
    let newAction = new Action(action);
    actions.push(newAction);
    idx = actions.length - 1;
    // insert into selected outcome if available
    var ob = viz.scene.activeObject.get();
    if (ob.action_id !== null && ob.outcome_key) {
      var nextAction;
      if (ob.action_id === 0) {
        nextAction = actions[0];
        actions[0] = idx;
      } else {
        nextAction = actions[ob.action_id].outcomes[ob.outcome_key];
        actions[ob.action_id].outcomes[ob.outcome_key] = idx;
      }
      if (nextAction) {
        let [action, skillDesc] = getActionAndSkillDesc(idx);
        action.outcomes[skillDesc.outcomes[0]] = nextAction;
      }
    }
    // update initial coordinate
    if (manualLayout) {
      viz.scene.updateRenderMetadata(newAction);

      let minY = viz.meta.manualScene.minY - viz.meta.translate[1] / viz.meta.scale;
      let minX = viz.meta.manualScene.minX - viz.meta.translate[0] / viz.meta.scale;

      if (ob.action_id !== null && ob.action_id !== 0 && ob.outcome_key) {
        let action = actions[ob.action_id];
        minX = action.pos[0] - newAction.renderMeta.width / 2;
        minY = action.pos[1] + action.renderMeta.height / 2 + 20;
      }

      let maxY = minY + newAction.renderMeta.height;
      let maxX = minX + newAction.renderMeta.width;

      let overlap = true;
      while (overlap) {
        overlap = false;
        for (let i = 1; i < actions.length - 1; i++) {
          let a = actions[i];
          let minAy = a.pos[1] - a.renderMeta.height / 2;
          let maxAy = a.pos[1] + a.renderMeta.height / 2;
          let minAx = a.pos[0] - a.renderMeta.width / 2;
          let maxAx = a.pos[0] + a.renderMeta.width / 2;
          if (
            axisIntercept(minY, maxY, minAy, maxAy) &&
            axisIntercept(minX, maxX, minAx, maxAx)
          ) {
            overlap = true;
            minX = maxAx + 10;
            maxX = minX + newAction.renderMeta.width;
            break;
          }
        }
      }

      let x = minX + newAction.renderMeta.width / 2 + 20;
      let y = minY + newAction.renderMeta.height / 2;
      newAction.pos = [x, y];
    }
    viz.scene.activeObject.set(idx, null);
    triggerDirty();
    return idx;
  }

  function updateAction(action, invalidateRender = true) {
    var idx = actions.indexOf(action);
    if (idx >= 0) {
      if (invalidateRender && manualLayout) {
        viz.scene.updateRenderMetadata(action);
      }
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeAction(action, suppressTriggerDirty) {
    push('removeAction');
    var idx;
    if (Number.isInteger(action)) {
      idx = action;
    } else {
      idx = actions.indexOf(action);
    }
    if (idx <= 0 || idx >= actions.length) {
      return;
    }

    actions.splice(idx, 1);

    // Adjust action indexes.
    var first = true;
    for (let action of actions) {
      if (first) {
        if (action === idx) {
          actions[0] = null;
        } else if (action > idx) {
          --actions[0];
        }
        first = false;
        continue;
      }
      for (let k in action.outcomes) {
        if (action.outcomes[k] === idx) {
          action.outcomes[k] = null;
        } else if (action.outcomes[k] > idx) {
          --action.outcomes[k];
        }
      }
    }
    if (!suppressTriggerDirty) {
      triggerDirty();
    }
  }

  function updateActionOutcome(action, key, toAction) {
    push('updateActionOutcome');
    var idx;
    if (Number.isInteger(action)) {
      idx = action;
    } else {
      idx = actions.indexOf(action);
    }
    if (idx < 0 || idx >= actions.length) {
      return;
    }

    var toIdx;
    if (Number.isInteger(toAction)) {
      toIdx = toAction;
    } else {
      toIdx = actions.indexOf(toAction);
    }
    if (toIdx < -outcomes.length || toIdx >= actions.length) {
      return;
    }

    if (idx === 0) {
      if (actions[0] !== toIdx) {
        actions[0] = toIdx;
        triggerDirty();
      }
    } else if (actions[idx].outcomes[key] !== toIdx) {
      actions[idx].outcomes[key] = toIdx;
      triggerDirty();
    }
  }

  function removeActionOutcome(action, key) {
    push('removeActionOutcome');
    var idx;
    if (Number.isInteger(action)) {
      idx = action;
    } else {
      idx = actions.indexOf(action);
    }
    if (idx < 0 || idx >= actions.length) {
      return;
    }
    if (idx === 0 && actions[0] !== null) {
      actions[0] = null;
      triggerDirty();
    } else if (actions[idx].outcomes[key] !== null) {
      actions[idx].outcomes[key] = null;
      triggerDirty();
    }
  }

  /* Manipulation: Cut, Copy & Paste */
  var cut = copy.bind(null, true);

  function copy(remove) {
    var ob = viz.scene.activeObject.get();
    if (ob.action_id && ob.action_id in actions) {
      var action = actions[ob.action_id];
      clipboard = {
        skillId: action.skillId,
        params: Object.assign({}, action.params)
      };
      if (remove) {
        removeAction(ob.action_id);
        viz.scene.activeObject.set(null, null);
      } else {
        viz.scene.activeObject.set(ob.action_id);
      }
      $viz.trigger('models.clipboard');
    }
  }

  function paste() {
    if (clipboard) {
      addAction({
        skillId: clipboard.skillId,
        params: Object.assign({}, clipboard.params)
      });
    }
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
      actions: JSON.stringify(actions),
      manualLayout: manualLayout,
      outcomes: outcomes
    });
    if (buffer.length > MAX_UNDO_LEVELS) {
      buffer.shift();
    }
  }

  function _restoreStructure(buffer) {
    var op = buffer.pop();
    var first = true;
    actions = _.map(JSON.parse(op.actions), function (action) {
      if (first) {
        first = false;
        if (Number.isInteger(action)) {
          return action;
        } else {
          return null;
        }
      }
      return new Action(action);
    });
    manualLayout = op.manualLayout;
    // TODO: this also has implication with top level options.
    outcomes = op.outcomes;
    viz.scene.activeObject.set(null, null);
    triggerDirty();
  }

  function setLayoutMode(isManualLayout) {
    push('setLayoutMode');
    if (manualLayout === isManualLayout) {
      return;
    }
    manualLayout = isManualLayout;
    if (manualLayout && !viz.scene.resetLayout()) {
      manualLayout = false;
      return;
    }
    triggerDirty();
  }

  function resetLayout() {
    if (!manualLayout) {
      return;
    }
    push('resetLayout');
    if (viz.scene.resetLayout()) {
      triggerDirty();
    }
  }

  function clearAll() {
    if (actions.length !== 1 || actions[0] !== null) {
      push('clearAll');
      actions = [null];
      viz.scene.activeObject.set(null, null);
      triggerDirty();
    }
  }

  function triggerDirty() {
    externalTriggerDirty();
    viz.modelsUpdated();
    $viz.trigger('models.updated');
  }

  function externalTriggerDirty() {
    if (!dirty) {
      dirty = true;
      $viz.trigger('models.dirty');
    }
  }

  function enableOverwrite() {
    // can only be set.
    if (overwrite) {
      return;
    }
    overwrite = true;
    triggerDirty();
  }

  function getActionAndSkillDesc(action_id) {
    if (action_id < 1 || action_id >= actions.length) {
      return null;
    }
    var action = actions[action_id];
    if (!action.skillId) {
      return null;
    }
    var skillDesc;
    if (action.skillId.indexOf('_ttpk_') === 0) {
      skillDesc = taskTemplateMetas[action.skillId];
    } else {
      skillDesc = skillDescriptions[action.skillId];
    }
    return [action, skillDesc];
  }

  /* Utils */
  function axisIntercept(minA, maxA, minB, maxB) {
    return maxA > minB && maxB > minA;
  }

  viz.models = {
    load: load,
    getMetadata: getMetadata,
    getStructure: getStructure,

    setTopLvl: setTopLvl,

    getCreateSuspended: () => createSuspended,
    setCreateSuspended: setCreateSuspended,

    addParam: addParam,
    updateParam: updateParam,
    moveDownParam: moveDownParam,
    moveUpParam: moveUpParam,
    removeParam: removeParam,
    updateOutcomes: updateOutcomes,

    addAction: addAction,
    updateAction: updateAction,
    removeAction: removeAction,
    updateActionOutcome: updateActionOutcome,
    removeActionOutcome: removeActionOutcome,

    setLayoutMode: setLayoutMode,
    resetLayout: resetLayout,

    cut: cut,
    copy: copy,
    paste: paste,

    canUndo: canUndo,
    canRedo: canRedo,
    undo: undo,
    redo: redo,
    push: push,

    clearAll: clearAll,
    externalTriggerDirty: externalTriggerDirty,
    enableOverwrite: enableOverwrite,

    getActionAndSkillDesc: getActionAndSkillDesc,

    id: () => id,
    topLvl: () => topLvl,
    rawParams: () => params,
    rawOutcomes: () => outcomes,
    rawActions: () => actions,
    rawSearch: () => search,
    skillDescriptions: () => skillDescriptions,
    taskTemplateMetas: () => taskTemplateMetas,
    stationList: () => stationList,
    registerList: () => registerList,
    globalParams: () => globalParams,
    variables: () => variables,
    graph: () => graph,
    isManualLayout: () => manualLayout,
    corrupted: () => corrupted,
    isDirty: () => dirty,
    overwrite: () => overwrite,
    isCached: () => cached
  };
}
