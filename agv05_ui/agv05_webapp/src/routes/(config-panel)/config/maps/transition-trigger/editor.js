import $ from 'cash-dom';

import dynformF from './dynform.js';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = dynformF(editor);

  modelsF(dynform);

  function load() {
    var transitionTrigger;
    var skillDescriptions;
    var taskTemplateMetas;
    var stationList;
    var registerList;
    var reservedGlobalParams;
    var globalParams;
    var variables;
    var filter;

    try {
      transitionTrigger = JSON.parse(data.tTrigger.value);
    } catch (e) {
      console.log('Transition Trigger data parse error.');
    }

    try {
      skillDescriptions = JSON.parse(data.meta.skill_descriptions);
    } catch (e) {
      console.log('Transition Trigger skill_descriptions parse error.');
    }

    try {
      taskTemplateMetas = JSON.parse(data.meta.tasktemplate_metas);
    } catch (e) {
      console.log('Transition Trigger tasktemplate_meta parse error.');
    }

    try {
      stationList = JSON.parse(data.meta.station_list);
    } catch (e) {
      console.log('Transition Trigger station_list parse error.');
    }

    try {
      registerList = JSON.parse(data.meta.register_list);
    } catch (e) {
      console.log('Transition Trigger register_list parse error.');
    }

    try {
      reservedGlobalParams = JSON.parse(data.meta.reserved_global_params);
    } catch (e) {
      console.log('Transition Trigger reserved_global_params parse error.');
    }

    try {
      globalParams = JSON.parse(data.meta.global_params);
    } catch (e) {
      console.log('Transition Trigger global_params parse error.');
    }

    try {
      variables = JSON.parse(data.meta.variables);
    } catch (e) {
      console.log('Transition Trigger variables parse error.');
    }

    let urlParams = new URL(window.location).searchParams;
    filter = urlParams.get('filter');

    dynform.models.load({
      transitionTrigger: transitionTrigger,
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
      value: JSON.stringify(dynform.models.getTransitionTrigger())
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
