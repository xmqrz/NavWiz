import $ from 'cash-dom';

import dynformF from './dynform.js';
import modelsF from './models';

export default function (outer, data) {
  var editor = $(outer);
  var dynform = dynformF(editor);
  editor.dynform = dynform;
  modelsF(dynform);

  function load() {
    var variables;
    var stationList;
    var filter;

    if (data.variable) {
      try {
        variables = JSON.parse(data.variable.value);
      } catch (e) {
        console.log('Variables data parse error.');
      }
    }

    if (data.meta) {
      try {
        stationList = JSON.parse(data.meta.station_list);
      } catch (e) {
        console.log('Variables station_list parse error.');
      }
    }

    let urlParams = new URL(window.location).searchParams;
    filter = urlParams.get('filter');

    dynform.models.load({
      variables: variables,
      stationList: stationList,
      filter: filter
    });
  }

  editor.stage = () => {
    return {
      value: JSON.stringify(dynform.models.getVariables())
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
