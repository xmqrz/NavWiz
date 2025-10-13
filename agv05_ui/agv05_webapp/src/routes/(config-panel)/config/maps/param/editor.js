import $ from 'cash-dom';

import editorF from '../../task-templates/global-param/editor.js';

export default function (outer, data) {
  var editor = $(outer);

  editor.on('ready', function () {
    // Monkey-patching global-param as a quick solution.
    // TODO: Refactor if there is a better way.
    var gpEditor = outer.editor;
    gpEditor.dynform.find('th').first().text('Map Parameters');
    gpEditor.dynform
      .find('.add-global-param')
      .on('click', limitParamTypes)
      .html('<i class="fa-solid fa-plus mr-2"></i> Add map parameter');
    gpEditor.find('.filter-container').remove();

    function limitParamTypes(event) {
      gpEditor.dynform.find('.param-input-type').each(function () {
        var input = $(this);
        var nonOptions = input.find('option').not('[value=double]');
        if (nonOptions.length) {
          nonOptions.remove();
          if (event) {
            input.trigger('change');
          }
        }
      });
    }
    limitParamTypes();
  });

  return editorF(outer, {
    globalParam: data.param,
    meta: data.meta
  });
}
