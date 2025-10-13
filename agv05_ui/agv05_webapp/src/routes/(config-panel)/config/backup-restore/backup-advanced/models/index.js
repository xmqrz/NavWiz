/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import treeF from '../../restore-advanced/models/tree';

export default function (form) {
  var rawTree = {};
  var tree = treeF();

  /* Deserialize from database */
  function load(options) {
    if (options.tree) {
      rawTree = options.tree;
    }

    tree.load(rawTree);

    //Signal model loaded
    form.loaded();
  }

  return {
    load: load,

    getStateTree: tree.getStateTree,
    getSelectedTree: tree.getSelectedTree,
    STATE: tree.STATE
  };
}
