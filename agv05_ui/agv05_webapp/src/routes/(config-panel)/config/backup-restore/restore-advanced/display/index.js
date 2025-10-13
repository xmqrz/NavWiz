/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import treeF from './tree';

var displayHtml = `
<div class="space-y-2">
  <div id="form-tree">
    <div class="controls tree-field"></div>
  </div>
</div>
`;

export default function (form) {
  let display = $(displayHtml);
  display.appendTo(form);
  display.treeField = display.find('.tree-field');

  let tree = treeF(form);

  function loaded() {
    display.treeField.empty();
    let treeView = tree.load();
    display.treeField.append(treeView);
  }

  return {
    loaded: loaded
  };
}
