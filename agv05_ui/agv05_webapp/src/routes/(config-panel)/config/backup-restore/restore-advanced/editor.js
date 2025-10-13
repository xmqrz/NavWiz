import $ from 'cash-dom';

import modelsF from './models';
import displayF from './display';

export default function (outer, data) {
  var editor = $(outer);

  function load() {
    let tree;

    try {
      tree = JSON.parse(data);
    } catch (e) {
      console.log('BackupAdvanced tree parse error.');
    }

    editor.models.load({
      tree: tree
    });
  }

  editor.stage = () => {
    return {
      value: JSON.stringify(editor.models.getSelectedTree())
    };
  };

  editor.models = modelsF(editor);
  editor.storage = {
    load: load
  };
  editor.display = displayF(editor);

  editor.loaded = function () {
    editor.display.loaded();
  };
  editor.storage.load();

  outer.editor = editor;
  editor.trigger('ready');
}
