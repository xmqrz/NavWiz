import $ from 'cash-dom';

import vizF from './viz';
import toolbarF from './toolbar';
import modelsF from './models';
import resizeHandleF from '$lib/shared/resize-handle';

export default function (outer, data) {
  var mapAnnotation = $(outer);
  var toolbar;

  var viz = vizF(mapAnnotation, {
    editable: mapAnnotation.attr('editable') === 'true',
    grid: mapAnnotation.attr('grid') === 'true'
  });
  var $viz = $(viz.node());

  if (mapAnnotation.attr('editable') === 'true') {
    toolbar = toolbarF(viz);
  }

  if (mapAnnotation.attr('resizable') === 'true') {
    resizeHandleF(viz.node());
  }

  function load() {
    var metadata, structure, stations, ocgChoices, ocg, annotations;

    try {
      metadata = JSON.parse(data.layout.metadata);
    } catch (e) {
      console.log('Map metadata parse error.');
    }

    try {
      structure = JSON.parse(data.layout.structure);
    } catch (e) {
      console.log('Map structure parse error.');
    }

    try {
      stations = JSON.parse(data.layout.stations);
    } catch (e) {
      console.log('Map stations parse error.');
    }

    try {
      ocgChoices = JSON.parse(data.layout.ocg_choices);
    } catch (e) {
      console.log('Map ocg_choices parse error.');
    }

    try {
      ocg = parseInt(data.layout.ocg);
    } catch (e) {
      console.log('Map ocg parse error.');
    }

    try {
      annotations = JSON.parse(data.annotation.annotations);
    } catch (e) {
      console.log('Map annotations parse error.');
    }

    viz.models.load({
      metadata: metadata,
      structure: structure,
      stations: stations,
      ocgChoices: ocgChoices,
      ocg: ocg,
      annotations: annotations
    });
  }

  function save() {
    mapAnnotation.trigger('save');
  }

  function stage() {
    return {
      annotations: JSON.stringify(viz.models.getAnnotation())
    };
  }

  // Prevent leaving dirty forms
  $viz.on('models.dirty', function () {
    mapAnnotation.trigger('dirty');
  });

  // Block default context menu
  mapAnnotation.on('contextmenu', function (event) {
    event.preventDefault();
  });

  viz.editor = {
    save: save,
    stage: stage
  };

  modelsF(viz);
  load();

  $viz.trigger('models.ready');
  outer.viz = viz;
  mapAnnotation.trigger('ready');
  $(window).on('resize', viz.resized);
  viz.resized();

  return {
    destroy() {
      $(window).off('resize', viz.resized);
      viz.destroy();
      if (toolbar) {
        toolbar.destroy();
      }
    }
  };
}
