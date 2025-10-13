import $ from 'cash-dom';

import vizF from './viz';
import toolbarF from './toolbar';
import dynformF from './dynform';
import modelsF from './models';
import resizeHandleF from '$lib/shared/resize-handle';

export default function (outer, [data, modalStore]) {
  var ocg = $(outer);
  var toolbar;

  var viz = vizF(ocg, {
    editable: ocg.attr('editable') === 'true',
    dynamic: ocg.attr('dynamic') === 'true',
    grid: ocg.attr('grid') === 'true'
  });
  viz.modalStore = modalStore;
  var $viz = $(viz.node());

  if (ocg.attr('editable') === 'true') {
    toolbar = toolbarF(viz);
  }

  if (ocg.attr('resizable') === 'true') {
    resizeHandleF(viz.node());
  }

  function load() {
    var ocgId, metadata, png;

    try {
      ocgId = parseInt(data.ocg.id);
    } catch (e) {
      console.log('Ocg id parse error.');
    }

    try {
      metadata = JSON.parse(data.ocg.metadata);
    } catch (e) {
      console.log('Ocg metadata parse error.');
    }

    png = new Image();
    png.onload = _load0;
    png.onerror = function () {
      console.log('Ocg png invalid.');
      if (png.src) {
        window.alert('Raw map image is corrupted.');
      } else {
        window.alert('Raw map image is missing.');
      }
      png = null;
      _load0();
    };
    png.src = data.ocg.png_file;

    function _load0() {
      viz.models.load({
        metadata: metadata,
        png: png,
        ocgId: ocgId
      });
      window.URL.revokeObjectURL(data.ocg.png_file);
    }
  }

  function save() {
    ocg.trigger('save');
  }

  function stage() {
    return {
      metadata: JSON.stringify(viz.models.getMetadata()),
      png_file: viz.models.getPng()
    };
  }

  // Prevent leaving dirty forms
  $viz.on('models.dirty', function () {
    ocg.trigger('dirty');
  });

  // Block default context menu
  ocg.on('contextmenu', function (event) {
    event.preventDefault();
  });

  viz.editor = {
    load: load,
    save: save,
    stage: stage
  };

  dynformF(viz);
  modelsF(viz);
  load();

  $viz.trigger('models.ready');
  outer.viz = viz;
  ocg.trigger('ready');
  $(window).on('resize', viz.resized);
  viz.resized();

  return {
    destroy() {
      viz.destroy();
      $(window).off('resize', viz.resized);
      if (toolbar) {
        toolbar.destroy();
      }
    }
  };
}
