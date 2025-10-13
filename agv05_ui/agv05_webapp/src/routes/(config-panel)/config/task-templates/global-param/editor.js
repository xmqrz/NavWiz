import $ from 'cash-dom';

import dynformF from './dynform.js';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = dynformF(editor);
  editor.dynform = dynform;
  modelsF(dynform);

  function load() {
    var reservedGlobalParams;
    var globalParams;
    var stationList;
    var registerList;
    var filter;

    if (data.globalParam) {
      try {
        globalParams = JSON.parse(data.globalParam.value);
      } catch (e) {
        console.log('GlobalParameter data parse error.');
      }
    }

    if (data.meta) {
      try {
        reservedGlobalParams = JSON.parse(data.meta.reserved_global_params);
      } catch (e) {
        console.log('GlobalParameter reserved data parse error.');
      }

      try {
        stationList = JSON.parse(data.meta.station_list);
      } catch (e) {
        console.log('GlobalParameter station_list parse error.');
      }

      try {
        registerList = JSON.parse(data.meta.register_list);
      } catch (e) {
        console.log('GlobalParameter register_list parse error.');
      }
    }

    let urlParams = new URL(window.location).searchParams;
    filter = urlParams.get('filter');

    dynform.models.load({
      reservedGlobalParams: reservedGlobalParams,
      globalParams: globalParams,
      stationList: stationList,
      registerList: registerList,
      filter: filter
    });
  }

  editor.stage = () => {
    return {
      value: JSON.stringify(dynform.models.getGlobalParams())
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
