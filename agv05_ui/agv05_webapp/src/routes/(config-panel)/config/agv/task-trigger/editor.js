/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import displayF from './display';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = displayF(editor);
  outer.taskTriggers = dynform;

  function load() {
    var taskTemplateMetas;
    var stationList;
    var registerList;
    var taskTriggersData;

    try {
      taskTemplateMetas = JSON.parse(data.tasktemplate_metas);
    } catch (e) {
      console.log('Agv task-trigger tasktemplate_meta parse error.');
    }

    try {
      stationList = JSON.parse(data.station_list);
    } catch (e) {
      console.log('Agv task-trigger station_list parse error.');
    }

    try {
      registerList = JSON.parse(data.register_list);
    } catch (e) {
      console.log('Agv task-trigger register_list parse error.');
    }

    try {
      taskTriggersData = JSON.parse(data.task_triggers);
    } catch (e) {
      console.log('Agv task-trigger task_triggers parse error.');
    }

    dynform.models.load({
      taskTemplateMetas: taskTemplateMetas,
      stationList: stationList,
      registerList: registerList,
      taskTriggers: taskTriggersData
    });
  }

  function clear() {
    data.task_triggers = '';
  }

  function stage() {
    data.task_triggers = JSON.stringify(dynform.models.getTaskTriggers());
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
