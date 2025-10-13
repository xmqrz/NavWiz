/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

var branchHtml = `
<div class="tree-node">
  <div class="tree-leaf mb-2">
    <button type="button" class="btn-sm text-sm py-1 px-2 rounded-lg tree-expend-btn variant-filled" aria-expanded="false">
      <i class="fa fa-plus"></i>
    </button>
    <label class="text-nowrap">
      <input type="checkbox" class="checkbox tree-input"/>
      <span class="leaf-text"></span>
    </label>
  </div>
  <div class="hidden tree-branch space-y-1" id="my-tree-branch">
  </div>
</div>
`;

var leafHtml = `
<div class="tree-node">
  <div class="tree-leaf leaf-end mb-2">
    <label class="text-nowrap">
      <input type="checkbox" class="checkbox tree-input"/>
      <span class="leaf-text"></span>
    </label>
  </div>
</div>
`;

export default function (form) {
  let STATE = form.models.STATE;
  let dispList = [];
  let branchCounter = 0;

  function load() {
    let stateTree = form.models.getStateTree();

    branchCounter = 0;
    dispList = [];

    let treeView = createTree(stateTree);
    treeView.leaf.find('.tree-expend-btn').css('visibility', 'hidden');
    treeView.branch.removeClass('hidden');

    // button visual
    treeView.find('.tree-expend-btn').on('click', function () {
      let state = $(this).attr('aria-expanded') === 'true';
      let icon = $(this).find('.fa');
      if (state) {
        $(this).attr('aria-expanded', false);
        $(this).parent().next('.tree-branch').addClass('hidden');
        icon.removeClass('fa-minus');
        icon.addClass('fa-plus');
      } else {
        $(this).attr('aria-expanded', true);
        $(this).parent().next('.tree-branch').removeClass('hidden');
        icon.removeClass('fa-plus');
        icon.addClass('fa-minus');
      }
    });

    return treeView;
  }

  function createTree(node) {
    let disp = node.branch ? createBranch(node) : createLeaf(node);
    dispList.push(disp);
    if (!node.children || !Array.isArray(node.children)) {
      return disp;
    }

    if (Array.isArray(node.children)) {
      node.children.forEach((c) => {
        disp.branch.append(createTree(c));
      });
    }
    return disp;
  }

  function createBranch(node) {
    let branch = $(branchHtml);
    let branchId = `node-${branchCounter}`;
    branchCounter++;
    branch.find('.tree-expend-btn').attr('href', `#${branchId}`);
    branch.find('.tree-branch').attr('id', branchId);

    branch.leaf = branch.children('.tree-leaf');
    branch.branch = branch.children('.tree-branch');
    branch.checkbox = branch.leaf.find('.tree-input');

    branch.leaf.find('.leaf-text').text(_.startCase(node.name));
    branch.checkbox.on('change', function () {
      let newState = branch.checkbox.is(':checked');
      if (node.state === STATE.INDETERMINATE) {
        newState = checkIndeterminateToggleState(node);
      }
      node.setState(newState);
      onStateChange();
    });
    branch.update = function () {
      applyCheckboxState(branch.checkbox, node.state);
    };
    branch.update();
    return branch;
  }

  function createLeaf(node) {
    let leaf = $(leafHtml);
    let textSpan = leaf.find('.leaf-text').text(node.name);
    leaf.checkbox = leaf.find('.tree-input');
    if (node.disabled === true) {
      leaf.checkbox.prop('disabled', true);
    } else {
      leaf.checkbox.on('change', function () {
        node.setState(leaf.checkbox.is(':checked'));
        onStateChange();
      });
    }
    if (node.depError) {
      textSpan.addClass('text-danger').attr('title', 'Missing dependency.');
    }
    leaf.update = function () {
      applyCheckboxState(leaf.checkbox, node.state);
    };
    leaf.update();
    return leaf;
  }

  function checkIndeterminateToggleState(node) {
    // if all checked except for disabled options the toggle state should be disabled.
    let state = false;

    function _checkState(n) {
      if (state === true) {
        return;
      }
      if (n.disabled === true) {
        return;
      }

      if (n.state === STATE.UNCHECKED) {
        state = true;
        return;
      }

      if (Array.isArray(n.children)) {
        n.children.forEach((c) => {
          _checkState(c);
        });
      }
    }
    _checkState(node);
    return state;
  }

  function applyCheckboxState(checkbox, state) {
    switch (state) {
      case STATE.UNCHECKED:
        checkbox.prop('checked', false).prop('indeterminate', false).removeAttr('style');
        break;
      case STATE.INDETERMINATE:
        checkbox.prop('checked', false).prop('indeterminate', true).removeAttr('style');
        break;
      case STATE.CHECKED:
        checkbox.prop('checked', true).prop('indeterminate', false).removeAttr('style');
        break;
      case STATE.FORCE_CHECKED:
        checkbox.prop('checked', true).prop('indeterminate', false).css('opacity', 0.25);
        break;
    }
  }

  function onStateChange() {
    dispList.forEach((d) => {
      d.update();
    });
  }

  return {
    load: load
  };
}
