import $ from 'cash-dom';

import vizF from './viz';
import toolbarF from './toolbar';
import modelsF from './models';
import resizeHandleF from '$lib/shared/resize-handle';
import maps from '$lib/shared/services/config/maps';

var ocgInputHtml = `
<select class="select"></select>
`;

export default function (outer, [data, modalStore]) {
  var map = $(outer);
  var mapID;
  var toolbar;

  var viz = vizF(map, {
    editable: map.attr('editable') === 'true',
    dynamic: map.attr('dynamic') === 'true',
    grid: map.attr('grid') === 'true',
    changesets: map.attr('changesets') === 'true'
  });
  viz.modalStore = modalStore;
  var $viz = $(viz.node());

  if (map.attr('editable') === 'true') {
    toolbar = toolbarF(viz);
  }

  if (map.attr('resizable') === 'true') {
    resizeHandleF(viz.node());
  }

  function load() {
    var ocgChoices, ocg, metadata, structure, stations, reservedParams, params, search;

    ocgInput.show();
    ocgChangesetInput.hide();

    mapID = data.layout.id;

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
      ocgChoices: ocgChoices,
      ocg: ocg,
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
      stations: JSON.stringify(viz.models.getStations().sort()),
      ocg: viz.models.ocgId()
    };
  }

  function loadChangeset(changeset) {
    ocgInput.hide();
    ocgChangesetInput.show();
    if (!changeset) {
      return;
    }
    maps.changeset(mapID, changeset.id).then(function (data) {
      try {
        var ocg, metadata, structure, stations;

        try {
          // if ocg does not exist anymore use current.
          ocg = parseInt(ocgChangesetInput.val(data.ocg).val());
        } catch (e) {
          console.log('Map ocg parse error.');
        }

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
          ocg: ocg,
          metadata: metadata,
          structure: structure,
          stations: stations
        });
      } catch (e) {
        console.log('Fail to apply map changeset');
      }
    });
  }

  // Map Ocg Options
  var ocgInput = $(ocgInputHtml);
  var ocgChangesetInput = $(ocgInputHtml);
  ocgInput.on('change', function () {
    viz.models.setOcg(parseInt(this.value));
    viz.liveApp.setTrackerInvalid();
  });
  ocgChangesetInput.prop('disabled', true).hide();
  map.append(ocgInput);
  map.append(ocgChangesetInput);
  function initOcgInput() {
    let ocg = viz.models.ocg();
    let ocgChoices = viz.models.rawOcgChoices();
    // TODO: sort based on changed time.
    for (const [_, o] of Object.entries(ocgChoices).sort().reverse()) {
      let opt = $(`<option value="${o.id}">${o.display_name}</option>`);
      ocgInput.append(opt);
    }
    if (ocg) {
      ocgInput.val(ocg.id);
    }
    ocgChangesetInput.html(ocgInput.html());
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
  initOcgInput();

  $viz.trigger('models.ready');
  outer.viz = viz;
  map.trigger('ready');
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
