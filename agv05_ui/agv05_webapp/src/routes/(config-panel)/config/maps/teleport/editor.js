import $ from 'cash-dom';

import dynformF from './dynform.js';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = dynformF(editor);

  modelsF(dynform);

  function load() {
    var teleport;
    var skillDescriptions;
    var taskTemplateMetas;
    var stationList;
    var registerList;
    var reservedGlobalParams;
    var globalParams;
    var variables;
    var filter;

    try {
      teleport = JSON.parse(data.teleport.value);
    } catch (e) {
      console.log('Teleport data parse error.');
    }

    try {
      skillDescriptions = JSON.parse(data.meta.skill_descriptions);
    } catch (e) {
      console.log('Teleport skill_descriptions parse error.');
    }

    try {
      taskTemplateMetas = JSON.parse(data.meta.tasktemplate_metas);
    } catch (e) {
      console.log('Teleport tasktemplate_meta parse error.');
    }

    try {
      stationList = JSON.parse(data.meta.station_list);
    } catch (e) {
      console.log('Teleport station_list parse error.');
    }

    try {
      registerList = JSON.parse(data.meta.register_list);
    } catch (e) {
      console.log('Teleport register_list parse error.');
    }

    try {
      reservedGlobalParams = JSON.parse(data.meta.reserved_global_params);
    } catch (e) {
      console.log('Teleport reserved_global_params parse error.');
    }

    try {
      globalParams = JSON.parse(data.meta.global_params);
    } catch (e) {
      console.log('Teleport global_params parse error.');
    }

    try {
      variables = JSON.parse(data.meta.variables);
    } catch (e) {
      console.log('Teleport variables parse error.');
    }

    let urlParams = new URL(window.location).searchParams;
    filter = urlParams.get('filter');

    dynform.models.load({
      teleport: teleport,
      skillDescriptions: skillDescriptions,
      taskTemplateMetas: taskTemplateMetas,
      stationList: stationList,
      registerList: registerList,
      reservedGlobalParams: reservedGlobalParams,
      globalParams: globalParams,
      variables: variables,
      filter: filter
    });
  }

  editor.stage = () => {
    return {
      value: JSON.stringify(dynform.models.getTeleport())
    };
  };

  dynform.on('models.dirty', function () {
    editor.trigger('dirty');
  });

  load();
  dynform.trigger('models.ready');
  outer.editor = editor;
  editor.trigger('ready');
}
