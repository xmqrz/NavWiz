/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

import treeF from './tree';

var MODE = ['restore', 'preserve', 'overwrite', 'rename'];
var IMPORT_BRANCH = ['map', 'task_template'];

export default function (form) {
  var rawTree = {};
  var tree = treeF();
  var restoreMode = true;

  /* Deserialize from database */
  function load(options) {
    if (options.tree) {
      rawTree = options.tree;
    }

    tree.load(rawTree);

    //Signal model loaded
    form.loaded();
  }

  function changeMode(mode) {
    if (!MODE.includes(mode)) {
      return;
    }

    // if mode does not change.
    if ((mode === 'restore') === restoreMode) {
      return;
    }

    if (mode === 'restore') {
      restoreMode = true;
      tree.load(rawTree);
    } else {
      restoreMode = false;
      tree.load(_.pick(rawTree, IMPORT_BRANCH));
    }

    //Signal model loaded
    form.loaded();
  }

  return {
    changeMode: changeMode,
    load: load,

    getStateTree: tree.getStateTree,
    getSelectedTree: tree.getSelectedTree,
    STATE: tree.STATE
  };
}
