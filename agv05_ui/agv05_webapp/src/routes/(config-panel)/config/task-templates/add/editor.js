import $ from 'cash-dom';

import vizF from './viz.js';
import dynformF from './dynform.js';
import modelsF from './models';
import toolbarF from './toolbar';
import searchF from './search';
import resizeHandleF from '$lib/shared/resize-handle';
import taskTemplates from '$lib/shared/services/config/task-templates';

var overwriteNoticeHtml = (id) => `
<div class="grid lg:grid-cols-2 py-2">
  <aside class="alert variant-filled-error">
    <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
    <div class="alert-message" data-toc-ignore="">
      <h3 class="h3" data-toc-ignore="">Warning</h3>
      <p>The content has been
        <a href="${taskTemplates.editUrl(id)}"
          target="_blank"
          class="font-bold">changed</a>
        while you are editing it.</p>
    </div>
    <div class="alert-actions">
      <button type="button" class="btn variant-filled force-save">Force overwrite</button>
    </div>
  </aside>
</div>
`;

export default function (outer, data) {
  var $outer = $(outer);
  var editable = $outer.attr('editable') === 'true';
  var preserve = $outer.attr('preserve') === 'true';
  var taskTemplateUrl = $outer.attr('taskTemplateUrl');
  var resizable = $outer.attr('resizable') === 'true';
  var overwrite = false;
  var toolbar;

  var viz = vizF({
    editable,
    preserve,
    taskTemplateUrl
  });
  var $viz = $(viz.node());

  var vizWrap = $('<div class="p-3">');
  $outer.append(vizWrap);
  vizWrap.append(viz.node());

  $viz.on('models.updated', () => {
    if (overwrite || !viz.models.overwrite() || !viz.models.id()) {
      return;
    }
    overwrite = true;
    var overwriteNotice = $(overwriteNoticeHtml(viz.models.id()));
    overwriteNotice.insertBefore(vizWrap);
    overwriteNotice.find('.force-save').on('click', () => {
      $outer.trigger('force-save');
    });
  });

  var dynform = dynformF(viz);
  $outer.prepend(dynform);

  if (editable) {
    toolbar = toolbarF(viz);
    searchF(viz);
  }

  if (resizable) {
    resizeHandleF(viz.node());
  }

  modelsF(viz);

  function load() {
    var id;
    var topLvl;
    var metadata;
    var structure;
    var skillDescriptions;
    var taskTemplateMetas;
    var stationList;
    var registerList;
    var reservedGlobalParams;
    var globalParams;
    var variables;
    var cached;
    var search;

    if (data.taskTemplate) {
      cached = data.taskTemplate.cached;
      id = data.taskTemplate.id;
      topLvl = !!data.taskTemplate.is_top_level;
      try {
        metadata = JSON.parse(data.taskTemplate.metadata);
      } catch (e) {
        console.log('TaskTemplate metadata parse error.');
      }

      try {
        structure = JSON.parse(data.taskTemplate.structure);
      } catch (e) {
        console.log('TaskTemplate structure parse error.');
      }
    }

    if (data.meta) {
      try {
        skillDescriptions = JSON.parse(data.meta.skill_descriptions);
      } catch (e) {
        console.log('TaskTemplate skill_descriptions parse error.');
      }

      try {
        taskTemplateMetas = JSON.parse(data.meta.tasktemplate_metas);
      } catch (e) {
        console.log('TaskTemplate tasktemplate_metas parse error.');
      }

      try {
        stationList = JSON.parse(data.meta.station_list);
      } catch (e) {
        console.log('TaskTemplate station_list parse error.');
      }

      try {
        registerList = JSON.parse(data.meta.register_list);
      } catch (e) {
        console.log('TaskTemplate register_list parse error.');
      }

      try {
        reservedGlobalParams = JSON.parse(data.meta.reserved_global_params);
      } catch (e) {
        console.log('TaskTemplate reserved_global_params parse error.');
      }

      try {
        globalParams = JSON.parse(data.meta.global_params);
      } catch (e) {
        console.log('TaskTemplate global_params parse error.');
      }

      try {
        variables = JSON.parse(data.meta.variables);
      } catch (e) {
        console.log('TaskTemplate variables parse error.');
      }
    }

    let urlParams = new URL(window.location).searchParams;
    search = urlParams.get('search');

    viz.models.load({
      id: id,
      topLvl: topLvl,
      metadata: metadata,
      structure: structure,
      skillDescriptions: skillDescriptions,
      taskTemplateMetas: taskTemplateMetas,
      stationList: stationList,
      registerList: registerList,
      reservedGlobalParams: reservedGlobalParams,
      globalParams: globalParams,
      variables: variables,
      cached: cached,
      search: search
    });
  }

  function save() {
    $outer.trigger('save');
  }

  function stage() {
    return {
      metadata: JSON.stringify(viz.models.getMetadata()),
      structure: JSON.stringify(viz.models.getStructure())
    };
  }

  // Prevent leaving dirty forms
  $viz.on('models.dirty', function () {
    $outer.trigger('dirty');
  });

  // Block default context menu
  $outer.on('contextmenu', function (event) {
    event.preventDefault();
  });

  viz.editor = {
    save: save,
    stage: stage
  };

  load();
  viz.init();
  $viz.trigger('models.ready');
  outer.viz = viz;
  $outer.trigger('ready');
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
