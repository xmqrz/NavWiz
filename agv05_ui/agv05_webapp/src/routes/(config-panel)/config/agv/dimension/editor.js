/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

import displayF from './display';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = displayF(editor);

  outer.dimension = dynform;

  function load() {
    var dimensionData;

    try {
      dimensionData = JSON.parse(data.dimension);
    } catch (e) {
      console.log('AGV dimension parse error.');
    }

    dynform.models.load(dimensionData || {});
  }

  function clear() {
    data.dimension = '';
  }

  function stage() {
    data.dimension = JSON.stringify(dynform.models.getDimension());
  }

  dynform.storage = {
    clear: clear,
    stage: stage
  };

  modelsF(outer);
  load();

  editor.clear = function () {
    dynform.storage.clear();
    dynform.empty();
  };
  editor.stage = dynform.storage.stage;
  outer.editor = editor;
  editor.trigger('ready');
}
