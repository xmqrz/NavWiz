import $ from 'cash-dom';

import vizF from './viz';
import toolbarF from './toolbar';
import modelsF from './models';
import resizeHandleF from '$lib/shared/resize-handle';
import maps from '$lib/shared/services/config/maps';

export default function (outer, data) {
  var map = $(outer);
  var mapID;
  var toolbar;

  var viz = vizF(map, {
    editable: map.attr('editable') === 'true',
    grid: map.attr('grid') === 'true',
    changesets: map.attr('changesets') === 'true'
  });
  var $viz = $(viz.node());

  if (map.attr('editable') === 'true') {
    toolbar = toolbarF(viz);
  }

  if (map.attr('resizable') === 'true') {
    resizeHandleF(viz.node());
  }

  function load() {
    var metadata, structure, stations, reservedParams, params, search;

    mapID = data.layout.id;

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
      reservedParams = JSON.parse(data.layout.reserved_params);
    } catch (e) {
      console.log('Map reserved_params parse error.');
    }

    try {
      params = JSON.parse(data.layout.params);
    } catch (e) {
      console.log('Map params parse error.');
    }

    if (viz.options.changesets) {
      maps.changesets(data.layout.id).then(function (response) {
        if (!response || !response.results) {
          return;
        }
        try {
          viz.models.loadChangesets({
            changesets: response.results
          });
        } catch (e) {
          console.log('Map changeset parse error.');
        }
      });
    }

    let urlParams = new URL(window.location).searchParams;
    search = urlParams.get('search');

    viz.models.load({
      metadata: metadata,
      structure: structure,
      stations: stations,
      reservedParams: reservedParams,
      params: params,
      search: search
    });
  }

  function save() {
    map.trigger('save');
  }

  function stage() {
    return {
      metadata: JSON.stringify(viz.models.getMetadata()),
      structure: JSON.stringify(viz.models.getStructure()),
      stations: JSON.stringify(viz.models.getStations().sort())
    };
  }

  function loadChangeset(changeset) {
    maps.changeset(mapID, changeset.id).then(function (data) {
      try {
        var metadata, structure, stations;

        try {
          metadata = JSON.parse(data.metadata);
        } catch (e) {
          console.log('Map metadata parse error.');
        }

        try {
          structure = JSON.parse(data.structure);
        } catch (e) {
          console.log('Map structure parse error.');
        }

        try {
          stations = JSON.parse(data.stations);
        } catch (e) {
          console.log('Map stations parse error.');
        }
        viz.models.load({
          metadata: metadata,
          structure: structure,
          stations: stations
        });
      } catch (e) {
        console.log('Fail to apply map changeset');
      }
    });
  }

  // Prevent leaving dirty forms
  $viz.on('models.dirty', function () {
    map.trigger('dirty');
  });

  // Block default context menu
  map.on('contextmenu', function (event) {
    event.preventDefault();
  });

  viz.editor = {
    load: load,
    save: save,
    stage: stage,
    loadChangeset: loadChangeset
  };

  modelsF(viz);
  load();

  $viz.trigger('models.ready');
  outer.viz = viz;
  map.trigger('ready');
  $(window).on('resize', viz.resized);
  viz.resized();

  return {
    destroy() {
      $(window).off('resize', viz.resized);
      if (toolbar) {
        toolbar.destroy();
      }
    }
  };
}
