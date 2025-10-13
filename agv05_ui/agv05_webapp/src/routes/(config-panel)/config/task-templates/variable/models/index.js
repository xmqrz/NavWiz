/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

import Variable from './variable';

export default function (dynform) {
  var variables = [];
  var stationList = [];
  var filter;
  var dirty = false;

  /* Deserialize from database */
  function load(options) {
    if (options.variables && Array.isArray(options.variables)) {
      variables = _.map(options.variables, (v) => new Variable(v));
    }
    if (options.stationList && Array.isArray(options.stationList)) {
      stationList = options.stationList;
    }
    if (options.filter) {
      filter = options.filter;
    }
  }

  /* Serialize to database */
  function getVariables() {
    return _.map(variables, (v) => _.pick(v, Variable.__attrs__));
  }

  /* Manipulation: Variable */
  function addVariable(v) {
    var idx = variables.indexOf(v);
    if (idx >= 0) {
      return idx;
    }
    variables.push(new Variable(v));
    triggerDirty();
    return variables.length - 1;
  }

  function updateVariable(v) {
    var idx = variables.indexOf(v);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function moveDownVariable(v) {
    var idx;
    if (Number.isInteger(v)) {
      idx = v;
    } else {
      idx = variables.indexOf(v);
    }
    if (idx < 0 || idx >= variables.length - 1) {
      return;
    }

    var tmp = variables[idx];
    variables[idx] = variables[idx + 1];
    variables[idx + 1] = tmp;
    triggerDirty();
    return true;
  }

  function moveUpVariable(v) {
    var idx;
    if (Number.isInteger(v)) {
      idx = v;
    } else {
      idx = variables.indexOf(v);
    }
    if (idx < 1 || idx >= variables.length) {
      return;
    }

    var tmp = variables[idx];
    variables[idx] = variables[idx - 1];
    variables[idx - 1] = tmp;
    triggerDirty();
    return true;
  }

  function removeVariable(v) {
    var idx;
    if (Number.isInteger(v)) {
      idx = v;
    } else {
      idx = variables.indexOf(v);
    }
    if (idx < 0 || idx >= variables.length) {
      return;
    }

    variables.splice(idx, 1);
    triggerDirty();
    return true;
  }

  /* Operation */
  function triggerDirty() {
    if (!dirty) {
      dirty = true;
      dynform.trigger('models.dirty');
    }
  }

  dynform.models = {
    load: load,
    getVariables: getVariables,

    addVariable: addVariable,
    updateVariable: updateVariable,
    moveDownVariable: moveDownVariable,
    moveUpVariable: moveUpVariable,
    removeVariable: removeVariable,

    externalTriggerDirty: triggerDirty,

    rawVariables: () => variables,
    stationList: () => stationList,
    rawFilter: () => filter,
    isDirty: () => dirty
  };
}
