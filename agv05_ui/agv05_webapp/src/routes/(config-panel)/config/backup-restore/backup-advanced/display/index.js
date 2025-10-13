/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import treeF from '../../restore-advanced/display/tree';

var displayHtml = `
<div class="space-y-2">
  <div id="form-tree" class="grid grid-cols-12">
    <label class="control-label col-span-2 requiredField">Check to backup<span class="asterikField">*</span></label>
    <div class="controls col-span-10 tree-field"></div>
  </div>
</div>
`;

export default function (form) {
  let display = $(displayHtml);
  display.insertAfter(form);
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
