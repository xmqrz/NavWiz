/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

var STATE = {
  UNCHECKED: 0,
  INDETERMINATE: 1,
  CHECKED: 2,
  FORCE_CHECKED: 3
};

export default function () {
  var tree = {};
  var stateTree = {};
  var depTree = {};

  function load(rawTree) {
    if (rawTree) {
      tree = rawTree;
    }
    stateTree = buildStateTree(tree, 'All');
    depTree = buildDepTree(stateTree);
  }

  function buildStateTree(tree, name, parent = undefined) {
    let node = {
      name: name,
      parent: parent,
      branch: true
    };
    if (Array.isArray(tree)) {
      node.children = tree.map(function (d) {
        let leaf = Object.assign(
          {
            parent: node,
            state: d.default ? STATE.CHECKED : STATE.UNCHECKED,
            branch: false
          },
          d
        );
        leaf.setState = getSetStateAction(leaf);
        leaf.setStateForDep = getSetStateForDep(leaf);
        return leaf;
      });
    } else if (tree) {
      node.children = [];
      Object.entries(tree)
        .sort()
        .forEach(([key, value]) => {
          node.children.push(buildStateTree(value, key, node));
        });
    }

    node.state = getState(node);
    node.setState = getSetStateAction(node);
    return node;
  }

  function getSetStateAction(node) {
    return function (state) {
      if (node.disabled === true) {
        return;
      }

      // Due to ui behaviour.
      if (node.state === STATE.FORCE_CHECKED) {
        node.state = STATE.CHECKED;
        return;
      }

      //State unchange
      if (
        (node.state === STATE.CHECKED && state) ||
        (node.state === STATE.UNCHECKED && !state)
      ) {
        return;
      }

      // Check dependency if setting state to false
      if (!state && node.gid && node.gid in depTree) {
        let required = false;
        depTree[node.gid].dep.forEach((d) => {
          if (d.state >= STATE.CHECKED) {
            required = true;
            return false;
          }
        });
        if (required) {
          node.state = STATE.FORCE_CHECKED;
          return;
        }
      }

      node.state = state ? STATE.CHECKED : STATE.UNCHECKED;

      //Update Dep
      if (Array.isArray(node.dep)) {
        node.dep.forEach((gid) => {
          if (!(gid in depTree) || !depTree[gid].node) {
            return;
          }
          depTree[gid].node.setStateForDep(state);
        });
      }

      //For top level operation.
      //Update Child
      if (Array.isArray(node.children) && node.children.length > 0) {
        node.children.forEach((c) => {
          c.setState(state);
        });
        node.state = getState(node);
      }

      //Update Parent
      let parent = node.parent;
      while (parent) {
        parent.state = getState(parent);
        parent = parent.parent;
      }
    };
  }

  function getSetStateForDep(node) {
    if (!node.gid) {
      return;
    }
    return function (state) {
      //State cannot change
      if (node.state === STATE.CHECKED) {
        return;
      }
      //State unchange
      if (
        (node.state === STATE.FORCE_CHECKED && state) ||
        (node.state === STATE.UNCHECKED && !state)
      ) {
        return;
      }

      //Check dep if setting state to false
      if (!state && node.gid && node.gid in depTree) {
        let required = false;
        depTree[node.gid].dep.forEach((d) => {
          if (d.state >= STATE.CHECKED) {
            required = true;
            return false;
          }
        });
        if (required) {
          node.state = STATE.FORCE_CHECKED;
          return;
        }
      }

      node.state = state ? STATE.FORCE_CHECKED : STATE.UNCHECKED;

      //Update Dep
      if (Array.isArray(node.dep)) {
        node.dep.forEach((gid) => {
          if (!(gid in depTree) || !depTree[gid].node) {
            return;
          }
          depTree[gid].node.setStateForDep(state);
        });
      }

      //Update Parent
      let parent = node.parent;
      while (parent) {
        parent.state = getState(parent);
        parent = parent.parent;
      }
    };
  }

  function getState(node) {
    if (!Array.isArray(node.children)) {
      return STATE.UNCHECKED;
    }
    let state = STATE.INDETERMINATE;
    let child = node.children.length;
    let checked = node.children.filter((c) => c.state >= STATE.CHECKED).length;
    let unchecked = node.children.filter((c) => c.state === STATE.UNCHECKED).length;
    if (child === checked) {
      state = STATE.CHECKED;
    } else if (child === unchecked) {
      state = STATE.UNCHECKED;
    }
    return state;
  }

  function buildDepTree(stateTree) {
    let dTree = {};
    _forEachStateNode(stateTree, function (n) {
      if (n.gid) {
        if (!(n.gid in dTree)) {
          dTree[n.gid] = {
            dep: []
          };
        }
        if (dTree[n.gid].node) {
          console.log('Backup advanced error: duplicate gid detected.');
        }
        dTree[n.gid].node = n;
      }

      if (Array.isArray(n.dep)) {
        n.dep.forEach((gid) => {
          if (!gid) {
            return;
          }
          if (!(gid in dTree)) {
            dTree[gid] = {
              dep: []
            };
          }
          dTree[gid].dep.push(n);
        });
      }
    });

    // Mark dep error for dep to missing node.
    Object.entries(dTree).forEach(function ([_key, value]) {
      if (value.node) {
        return;
      }
      value.dep.forEach(function (n) {
        n.depError = true;
      });
    });
    return dTree;
  }

  function _forEachStateNode(node, callback) {
    if (node.children && node.children.length > 0) {
      node.children.forEach((n) => {
        _forEachStateNode(n, callback);
      });
    }
    callback(node);
  }

  function getSelectedTree() {
    return processSelectedTree(stateTree, copyTree());
  }

  function processSelectedTree(stateData, data) {
    if (Array.isArray(data)) {
      for (let i = data.length - 1; i >= 0; i--) {
        if (stateData.children[i].state === STATE.UNCHECKED) {
          data.splice(i, 1);
        }
      }
    } else if (data) {
      stateData.children.forEach((c) => {
        if (!c.name) {
          return;
        }
        if (c.state === STATE.UNCHECKED) {
          delete data[c.name];
        } else {
          data[c.name] = processSelectedTree(c, data[c.name]);
          if (_.isEmpty(data[c.name])) {
            delete data[c.name];
          }
        }
      });
    }

    return data;
  }

  function copyTree() {
    return JSON.parse(JSON.stringify(tree));
  }

  return {
    getTree: () => tree,
    getStateTree: () => stateTree,
    getSelectedTree: getSelectedTree,
    load: load,
    STATE: STATE
  };
}
