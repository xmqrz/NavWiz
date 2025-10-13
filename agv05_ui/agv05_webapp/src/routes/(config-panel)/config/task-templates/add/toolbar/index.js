/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import Select from 'components/Select.svelte';
import { popup } from '@skeletonlabs/skeleton';

import Action from '../models/action.js';
import EditMode from '../scene/edit-mode.js';

import markerF from './marker';

/* Toolbar template */
var toolbarHtml =
  `
<div class="agv05-toolbar">
` +
  /* Group 0: Save */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-save !variant-filled-primary" disabled tip-title="Save">
    <i class="fa-solid fa-floppy-disk"></i>
  </button>
</div>
` +
  /* Group 1: Layout, Pointer & Link */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-layout-reset active" style="pointer-events:none" tip-title="Reset Layout">
    <i class="fa-solid fa-sitemap"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-layout">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-layout-popup">
  <ul class="rounded bg-surface-200-700-token">
    <li><button type="button" class="btn toolbar-layout-auto w-full">
      <i class="fa fa-lock mr-2 w-5"></i>
      Auto Layout
    </button></li>
    <li><button type="button" class="btn toolbar-layout-manual w-full">
      <i class="fa fa-unlock mr-2 w-5"></i>
      Manual Layout
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-pointer active" tip-title="Select (Esc)">
    <i class="fa fa-mouse-pointer"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-link active" tip-title="Link (F2)">
    <i class="fa fa-link"></i>
  </button>
</div>
` +
  /* Group 2: Add Action */
  `
<div class="inline-flex h-[43px]">
  <button type="button" class="btn rounded toolbar-add-action rounded-r-none border-r border-gray-400/50" tip-title="Hint: Select an outcome first to directly insert the action into the outcome.">
    <i class="fa fa-plus"></i>
    Add action:
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-swap-action-dropdown border-r border-gray-400/50">
    <i class="fa-solid fa-caret-down"></i>
  </button>
  <div class="toolbar-action inline-flex"></div>
</div>
<div class="z-50" data-popup="toolbar-swap-action-popup">
  <ul class="rounded bg-surface-200-700-token">
    <li><button type="button" class="btn toolbar-swap-action w-full">
      <i class="fa fa-exchange mr-2 w-5"></i>
      Swap action (Ctrl+Q)
    </button></li>
  </ul>
</div>
` +
  /* Group 3: Cut, Copy, Paste, Undo, Redo & Clear Everything */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-clear-all" tip-title="Clear Everything">
    <i class="fa fa-eraser"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-utilities" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false">
    <i class="fa fa-ellipsis-h"></i>
  </button>
</div>
<ul class="z-50 rounded bg-gray-400 border border-gray-400" data-popup="toolbar-utilities-popup">
  <li><div role="toolbar" aria-label="...">
    <div class="btn-group rounded">
      <button type="button" class="btn-icon rounded-none toolbar-cut" tip-title="Cut (Ctrl+X)" disabled="disabled">
        <i class="fa-solid fa-scissors fa-rotate-270"></i>
      </button>
      <button type="button" class="btn-icon rounded-none toolbar-copy" tip-title="Copy (Ctrl+C)" disabled="disabled">
        <i class="fa-solid fa-copy"></i>
      </button>
      <button type="button" class="btn-icon rounded-none toolbar-paste" tip-title="Paste (Ctrl+V)" disabled="disabled">
        <i class="fa-solid fa-clipboard"></i>
      </button>
    </div>
    <div class="btn-group rounded">
      <button type="button" class="btn-icon rounded-none toolbar-undo" tip-title="Undo (Ctrl+Z)" disabled="disabled">
        <i class="fa-solid fa-rotate-left"></i>
      </button>
      <button type="button" class="btn-icon rounded-none toolbar-redo" tip-title="Redo (Ctrl+Y)" disabled="disabled">
        <i class="fa-solid fa-rotate-right"></i>
      </button>
    </div>
  </div></li>
</ul>
` +
  /* Group 4: Zooming */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-zoom-in" tip-title="Zoom In">
    <i class="fa fa-search-plus"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zoom-out" tip-title="Zoom Out">
    <i class="fa fa-search-minus"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zoom-fit" tip-title="Fit Content">
    <i class="fa fa-crosshairs"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-fullscreen-hide" tip-title="Exit Fullscreen">
    <i class="fa fa-compress"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-fullscreen-show" tip-title="Show Fullscreen">
    <i class="fa fa-expand"></i>
  </button>
</div>
` +
  /* Group 5: Selection properties */
  `
<span class="divider-vertical toolbar-action-properties"></span>
<div class="inline-block toolbar-action-properties">Action: </div>
<div class="btn-group toolbar-action-properties rounded">
  <div class="toolbar-action-tooltip">
    <button type="button" class="btn-icon rounded-none toolbar-action-tooltip-btn" tip-title="Edit Notes">
      <i class="fa fa-pencil-square"></i>
    </button>
  </div>
  <button type="button" class="btn-icon rounded-none toolbar-action-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-action-tooltip-popup">
  <ul class="bg-surface-100-800-token rounded p-4">
    <li>
      <label class="label text-token">Notes:</label>
    </li>
    <li>
      <textarea class="textarea text-token p-4 toolbar-action-tooltip-textarea whitespace-pre resize" rows="5"></textarea>
    </li>
  </ul>
</div>
<span class="divider-vertical toolbar-param-properties"></span>
<div class="inline-block toolbar-param-properties">Param: </div>
<div class="btn-group toolbar-param-properties rounded">
  <button type="button" class="btn-icon rounded-none toolbar-param-prev" tip-title="Prev (Shift+Tab)">
    <i class="fa fa-arrow-up"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-param-next" tip-title="Next (Tab)">
    <i class="fa fa-arrow-down"></i>
  </button>
</div>
<div class="inline-flex">
  <div class="toolbar-param-properties input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black toolbar-param-label">Label</div>
    <label class="px-3 toolbar-param-input-checkbox-disp m-auto">
      <input type="checkbox" class="checkbox toolbar-param-input toolbar-param-input-checkbox" tip-title="Description"/>
      <span class="text-black">true</span>
    </label>
    <input type="text" class="toolbar-param-input toolbar-param-input-text" tip-title="Description"/>
    <input type="number" class="toolbar-param-input toolbar-param-input-number" tip-title="Description"/>
    <select class="toolbar-select toolbar-param-input toolbar-param-input-station" tip-title="Description"></select>
    <select class="toolbar-select toolbar-param-input toolbar-param-input-register" tip-title="Description"></select>
    <select class="toolbar-select toolbar-param-input toolbar-param-input-param" tip-title="Description"></select>
    <select class="toolbar-select toolbar-param-input toolbar-param-input-variable" tip-title="Description"></select>
    <div class="input-group-shim" style="padding: 0px">
      <button type="button" id="toolbar-marker-type-config-btn" class="btn-icon rounded-none border-r border-gray-400/50" style="justify-content: center" tip-title="Marker Type Configuration">
        <i class="fa fa-info-circle"></i>
      </button>
      <label tip-title="Inherit Template Parameter" class="text-black flex items-center gap-2 px-3">
        <input type="checkbox" class="checkbox toolbar-param-inherit"></input>
        <span>Inherit</span>
      </label>
    </div>
  </div>
</div>
<span class="divider-vertical toolbar-link-properties"></span>
<div class="inline-block toolbar-link-properties">Link: </div>
<div class="btn-group toolbar-link-properties rounded">
  <button type="button" class="btn-icon rounded-none toolbar-link-remove" tip-title="Delete">
    <i class="fa fa-trash"></i>
  </button>
</div>
<div class="toolbar-marker-parent"></div>
` +
  /* Group 6: Misc */
  `
<div class="toolbar-search-tools text-nowrap inline-block">
  <span class="divider-vertical toolbar-search-tools"></span>
  <div class="inline-block toolbar-search-tools">Search: </div>
  <div class="toolbar-search-tools inline-flex">
    <div class="toolbar-search-field inline-flex"></div>
    <button type="button" class="btn rounded rounded-none border-l border-gray-400/50 toolbar-search-count" disabled>-/-</a>
    <button type="button" class="btn rounded rounded-none border-l border-gray-400/50 toolbar-search-prev" tip-title="Prev">
      <i class="fa fa-arrow-up"></i>
    </button>
    <button type="button" class="btn rounded rounded-none border-l border-gray-400/50 toolbar-search-next" tip-title="Next">
      <i class="fa fa-arrow-down"></i>
    </button>
    <button type="button" class="btn rounded rounded-l-none border-l border-gray-400/50 toolbar-search-close" tip-title="Close">
      <i class="fa fa-times"></i>
    </button>
  </div>
</div>
` +
  /* End tag */
  `
</div>
`;

export default function (viz) {
  var toolbar = $(toolbarHtml);
  var $viz = $(viz.node());

  // NOTE: Svelte Select component required dom to be mount first.
  toolbar.insertBefore($viz);

  var titleInfo = $viz.parent().siblings('.title-info');

  /* Group 0: Save */
  var save = toolbar.find('.toolbar-save').on('click', function (e) {
    hideFullscreen();
    viz.editor.save();
    e.preventDefault();
  });

  $viz.on('models.dirty', function () {
    save.removeAttr('disabled');
  });

  /* Group 1: Layout, Pointer & Link */
  var layout = toolbar.find('.toolbar-layout');
  popup(layout[0], {
    event: 'click',
    target: 'toolbar-layout-popup',
    placement: 'bottom'
  });

  var resetLayout = toolbar.find('.toolbar-layout-reset').on('click', (e) => {
    viz.models.resetLayout();
    e.preventDefault();
  });
  toolbar.find('.toolbar-layout-auto').on('click', (e) => {
    viz.models.setLayoutMode(false);
    e.preventDefault();
  });
  toolbar.find('.toolbar-layout-manual').on('click', (e) => {
    viz.models.setLayoutMode(true);
    e.preventDefault();
  });

  $viz.on('models.ready', updateLayoutToolbar);
  $viz.on('models.updated', updateLayoutToolbar);

  function updateLayoutToolbar() {
    let manual = viz.models.isManualLayout();
    if (manual) {
      resetLayout.removeClass('active');
      resetLayout.attr('style', '');
    } else {
      resetLayout.addClass('active');
      resetLayout.attr('style', 'pointer-events:none');
    }
  }

  var pointer = toolbar.find('.toolbar-pointer').on('click', activatePointer);
  var link = toolbar.find('.toolbar-link').on('click', activateLink);

  function activatePointer(e) {
    pointer.addClass('active');
    link.removeClass('active');
    viz.scene.setEditMode(EditMode.POINTER);
    if (e) {
      e.preventDefault();
    }
  }

  function activateLink(e) {
    link.addClass('active');
    pointer.removeClass('active');
    viz.scene.setEditMode(EditMode.LINK);
    if (e) {
      e.preventDefault();
    }
  }

  $viz.on('scene.editMode', function () {
    switch (viz.scene.mode) {
      case EditMode.POINTER:
        activatePointer();
        break;
      case EditMode.LINK:
        activateLink();
        break;
    }
  });

  function onKeyupAO(event) {
    if ($(event.target).is('input')) {
      return;
    }
    if (event.keyCode === 0x1b) {
      // Escape
      if (viz.scene.mode !== EditMode.POINTER) {
        activatePointer();
      } else {
        viz.scene.activeObject.set(null, null);
      }
    } else if (event.keyCode === 0x71) {
      // F2
      activateLink();
    } else if (event.keyCode === 0x2e) {
      // Delete
      viz.scene.activeObject.remove();
    }
  }
  $(document.body).on('keyup', onKeyupAO);

  activatePointer();

  /* Group 2: Add Action */
  var actionTarget = toolbar.find('.toolbar-action');
  var action = new Select({
    target: actionTarget[0],
    props: {
      buttonClass: 'rounded rounded-l-none'
    }
  });
  toolbar.find('.toolbar-add-action').on('click', function (e) {
    var skillId = action.value;
    var skillDesc;
    if (skillId.indexOf('_ttpk_') === 0) {
      skillDesc = viz.models.taskTemplateMetas()[skillId];
    } else {
      skillDesc = viz.models.skillDescriptions()[skillId];
    }
    var params = {};
    for (let p of skillDesc.params) {
      params[p.key] = p.default;
    }
    viz.models.addAction(
      new Action({
        skillId: skillId,
        params: params
      })
    );
    e.preventDefault();
  });
  var swapActionDropdown = toolbar.find('.toolbar-swap-action-dropdown');
  popup(swapActionDropdown[0], {
    event: 'click',
    target: 'toolbar-swap-action-popup',
    placement: 'bottom'
  });
  toolbar.find('.toolbar-swap-action').on('click', swapAction);

  function swapAction() {
    if (!viz.scene.activeObject.isAction()) {
      return;
    }
    var ob = viz.scene.activeObject.get();
    var [_action, _skillDesc] = viz.models.getActionAndSkillDesc(ob.action_id);

    var skillId = action.value;
    if (_action.skillId === skillId) {
      return;
    }
    var skillDesc;
    if (skillId.indexOf('_ttpk_') === 0) {
      skillDesc = viz.models.taskTemplateMetas()[skillId];
    } else {
      skillDesc = viz.models.skillDescriptions()[skillId];
    }
    var params = {};
    for (let p of skillDesc.params) {
      params[p.key] = p.default;
      for (let p2 of _skillDesc.params) {
        if (p.name === p2.name) {
          if (p.type === p2.type && p2.key in _action.params) {
            params[p.key] = _action.params[p2.key];
          }
          break;
        }
      }
    }
    var _outcomes = Object.assign({}, _action.outcomes);
    var outcomes = {};
    for (let o of skillDesc.outcomes) {
      if (o in _outcomes) {
        outcomes[o] = _outcomes[o];
        delete _outcomes[o];
      }
    }
    var _i = 0;
    for (let o of skillDesc.outcomes) {
      if (o in outcomes) {
        continue;
      }
      let o2 = _skillDesc.outcomes[_i];
      if (o2 && o2 in _outcomes) {
        outcomes[o] = _outcomes[o2];
      }
      ++_i;
    }

    viz.models.push('swapAction');
    _action.skillId = skillId;
    _action.params = params;
    _action.outcomes = outcomes;
    viz.models.updateAction(_action);
  }

  $viz.on('scene.activeObject', function () {
    swapActionDropdown.toggle(viz.scene.activeObject.isAction());
  });

  $viz.on('models.ready', function () {
    var skillDescriptions = viz.models.skillDescriptions();
    var taskTemplateMetas = viz.models.taskTemplateMetas();

    var actionOptions = [];

    var opts = {
      name: 'Built-in',
      options: []
    };
    for (let k in skillDescriptions) {
      let d = skillDescriptions[k];
      opts.options.push([d.id, d.name]);
    }
    actionOptions.push(opts);

    var opts1 = {
      name: 'Task Template',
      options: []
    };
    var opts2 = {
      name: 'Task Template (Top-level)',
      options: []
    };
    for (let k in taskTemplateMetas) {
      let d = taskTemplateMetas[k];
      if (!d.is_top_level) {
        opts1.options.push([d.id, d.name, !d.is_active]);
      } else {
        opts2.options.push([d.id, d.name, !d.is_active]);
      }
    }
    actionOptions.push(opts1);
    actionOptions.push(opts2);
    action.options = actionOptions;
  });

  function onKeyupA1(event) {
    if ($(event.target).is('input')) {
      return;
    }
    if (event.ctrlKey) {
      if (event.keyCode === 0x51) {
        // Ctrl+Q
        swapAction();
      }
    }
  }
  $(document.body).on('keyup', onKeyupA1);

  /* Group 3: Cut, Copy, Paste, Undo, Redo & Clear Everything */
  var utils = toolbar.find('.toolbar-utilities');
  popup(utils[0], {
    event: 'click',
    target: 'toolbar-utilities-popup',
    placement: 'bottom'
  });

  var cut = toolbar.find('.toolbar-cut').on('click', () => viz.models.cut());
  var copy = toolbar.find('.toolbar-copy').on('click', () => viz.models.copy());
  var paste = toolbar.find('.toolbar-paste').on('click', () => viz.models.paste());
  var undo = toolbar.find('.toolbar-undo').on('click', () => viz.models.undo());
  var redo = toolbar.find('.toolbar-redo').on('click', () => viz.models.redo());
  toolbar.find('.toolbar-clear-all').on('click', () => viz.models.clearAll());

  $viz.on('scene.activeObject', function () {
    var ob = viz.scene.activeObject.get();
    cut.attr('disabled', ob.action_id ? null : 'disabled');
    copy.attr('disabled', ob.action_id ? null : 'disabled');
  });

  $viz.on('models.clipboard', function () {
    activatePointer();
    paste.removeAttr('disabled');
  });

  $viz.on('models.undoBuffer', function () {
    undo.attr('disabled', viz.models.canUndo() ? null : 'disabled');
    redo.attr('disabled', viz.models.canRedo() ? null : 'disabled');
  });

  let onCut = (e) => ($(e.target).is('input') ? null : viz.models.cut());
  let onCopy = (e) => ($(e.target).is('input') ? null : viz.models.copy());
  let onPaste = (e) => ($(e.target).is('input') ? null : viz.models.paste());

  $(document).on('cut', onCut);
  $(document).on('copy', onCopy);
  $(document).on('paste', onPaste);

  function onKeyup(event) {
    if ($(event.target).is('input')) {
      return;
    }
    if (event.ctrlKey) {
      if (event.keyCode === 0x5a) {
        // Ctrl+Z
        viz.models.undo();
      } else if (event.keyCode === 0x59) {
        // Ctrl+Y
        viz.models.redo();
      }
    }
  }
  $(document.body).on('keyup', onKeyup);

  /* Group 4: Zooming */
  toolbar.find('.toolbar-zoom-in').on('click', function (e) {
    viz.zoom.zoomIn();
    e.preventDefault();
  });
  toolbar.find('.toolbar-zoom-out').on('click', function (e) {
    viz.zoom.zoomOut();
    e.preventDefault();
  });
  toolbar.find('.toolbar-zoom-fit').on('click', function (e) {
    viz.zoom.zoomFit();
    e.preventDefault();
  });
  var fullscreenHide = toolbar.find('.toolbar-fullscreen-hide').on('click', hideFullscreen);
  var fullscreenShow = toolbar.find('.toolbar-fullscreen-show').on('click', showFullscreen);
  var fullscreenRestore;

  function hideFullscreen() {
    if (!fullscreenRestore) {
      return;
    }
    $viz.removeClass('agv05-fullscreen');
    toolbar.removeClass('agv05-fullscreen');
    titleInfo.removeClass('agv05-fullscreen');
    $viz.height(fullscreenRestore.height);
    let pageDom = document.querySelector('#page') || {};
    pageDom.scrollLeft = fullscreenRestore.scrollLeft;
    pageDom.scrollTop = fullscreenRestore.scrollTop;
    fullscreenRestore = null;

    fullscreenShow.insertAfter(fullscreenHide);
    fullscreenShow.show();
    fullscreenHide.hide();
    viz.resized();
  }

  function showFullscreen() {
    if (fullscreenRestore) {
      return;
    }
    let pageDom = document.querySelector('#page') || {};
    fullscreenRestore = {
      scrollLeft: pageDom.scrollLeft || 0,
      scrollTop: pageDom.scrollTop || 0,
      height: $viz.height()
    };
    $viz.css('height', '');
    $viz.addClass('agv05-fullscreen');
    toolbar.addClass('agv05-fullscreen');
    titleInfo.addClass('agv05-fullscreen');

    fullscreenHide.insertAfter(fullscreenShow);
    fullscreenHide.show();
    fullscreenShow.hide();
    viz.resized();
  }

  fullscreenHide.hide();

  /* Group 5: Selection Properties */
  var markerModal = markerF(toolbar.find('.toolbar-marker-parent'));
  var actionProperties = toolbar.find('.toolbar-action-properties');
  var paramProperties = toolbar.find('.toolbar-param-properties');
  var linkProperties = toolbar.find('.toolbar-link-properties');

  var actionTooltip = toolbar.find('.toolbar-action-tooltip');
  var actionTooltipBtn = actionTooltip.find('.toolbar-action-tooltip-btn');
  popup(actionTooltipBtn[0], {
    event: 'click',
    target: 'toolbar-action-tooltip-popup',
    placement: 'bottom',
    state: popupTooltipState
  });
  var actionTooltipTextarea = toolbar
    .find('.toolbar-action-tooltip-textarea')
    .on('blur', tooltipChanged);

  toolbar.find('.toolbar-action-remove').on('click', viz.scene.activeObject.remove);
  var paramPrev = toolbar.find('.toolbar-param-prev').on('click', activateParamPrev);
  var paramNext = toolbar.find('.toolbar-param-next').on('click', activateParamNext);
  var paramLabel = toolbar.find('.toolbar-param-label');
  var paramInputs = toolbar.find('.toolbar-param-input');
  var paramInputCheckboxDisp = toolbar.find('.toolbar-param-input-checkbox-disp');
  var paramInputCheckbox = toolbar.find('.toolbar-param-input-checkbox');
  var paramInputText = toolbar.find('.toolbar-param-input-text');
  var paramInputNumber = toolbar.find('.toolbar-param-input-number');
  var paramInputStation = toolbar.find('.toolbar-param-input-station');
  var paramInputRegister = toolbar.find('.toolbar-param-input-register');
  var paramInputParam = toolbar.find('.toolbar-param-input-param');
  var paramInputVariable = toolbar.find('.toolbar-param-input-variable');
  var paramInherit = toolbar.find('.toolbar-param-inherit');
  toolbar.find('.toolbar-link-remove').on('click', viz.scene.activeObject.remove);
  var markerTypeConfigBtn = toolbar.find('#toolbar-marker-type-config-btn').on('click', () => {
    let initValue = paramInputText.val();
    markerModal.show(initValue);
  });

  markerModal.on('apply', (e, value) => {
    paramInputText.val(value);
    paramInputs.trigger('mouseout').trigger('blur');
    viz.models.push('updateParam');
    let paramKey = _param_skillDesc.params[_param_idx].key;
    _param_action.params[paramKey] = value;
    viz.models.updateAction(_param_action);
  });

  $viz.on('models.ready', function () {
    var stationList = viz.models.stationList();
    paramInputStation.html('');
    for (let s of stationList) {
      let option = $('<option>');
      option.val(s).text(s);
      paramInputStation.append(option);
    }

    var registerList = viz.models.registerList();
    paramInputRegister.html('');
    for (let r of registerList) {
      let option = $('<option>');
      option.val(r).text(r);
      paramInputRegister.append(option);
    }
  });

  var _param_action;
  var _param_skillDesc;
  var _param_idx;

  function popupTooltipState(e) {
    if (e.state) {
      actionTooltipTextarea.val(_param_action.tooltip || '');
    }
  }

  function tooltipChanged() {
    viz.models.push('updateTooltip');
    _param_action.tooltip = actionTooltipTextarea.val();
    viz.models.updateAction(_param_action, false);
  }

  function activateParamPrev() {
    activateParam(_param_idx - 1);
  }

  function activateParamNext() {
    activateParam(_param_idx + 1);
  }

  function activateParamZeroOrHide() {
    if (!viz.scene.activeObject.isAction()) {
      paramProperties.hide();
      paramInputs.off('change');
      paramInputNumber.val('');
      markerModal.hide();
      return;
    }
    var ob = viz.scene.activeObject.get();
    var res = viz.models.getActionAndSkillDesc(ob.action_id);
    _param_action = res[0];
    _param_skillDesc = res[1];

    if (_param_skillDesc.params.length > 0) {
      activateParam(0);
    } else {
      _param_idx = null;
      paramProperties.hide();
      paramInputs.off('change');
      paramInputNumber.val('');
      markerModal.hide();
    }
  }

  function activateParam(param_id) {
    if (!(_param_action && _param_skillDesc)) {
      return;
    }
    if (param_id < 0 || param_id >= _param_skillDesc.params.length) {
      return;
    }
    var formatLabel = _param_skillDesc.id.indexOf('_ttpk_') === 0 ? _.identity : _.startCase;
    var paramDesc = _param_skillDesc.params[param_id];
    var label = formatLabel(paramDesc.name);
    if (paramDesc.version) {
      label += ' (v' + paramDesc.version + ')';
    }
    paramLabel.text(label);
    _param_idx = param_id;
    if (_param_idx <= 0) {
      paramPrev.attr('disabled', 'disabled');
    } else {
      paramPrev.removeAttr('disabled');
    }
    if (_param_idx >= _param_skillDesc.params.length - 1) {
      paramNext.attr('disabled', 'disabled');
    } else {
      paramNext.removeAttr('disabled');
    }
    setupParamInput();
    paramProperties.show();
    markerTypeConfigBtn.hide();
    if (paramDesc.name === 'marker_type' && paramDesc.type === 'str') {
      markerTypeConfigBtn.show();
    }
  }

  function setupParamInput() {
    var paramDesc = _param_skillDesc.params[_param_idx];
    var paramVal = _param_action.params[paramDesc.key];
    paramInputs.hide().off('change');
    paramInputCheckboxDisp.hide();
    paramInputNumber.val('');
    paramInherit.prop('checked', false).off('change').on('change', updateParamInherit);

    if (_.isString(paramVal) && paramVal.indexOf('${') === 0) {
      paramInherit.prop('checked', true);
      populateParamInputParam(paramDesc.type);
      paramInputParam
        .attr('tip-title', paramDesc.description)
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    } else if (paramDesc.type === 'bool') {
      paramInputCheckboxDisp.show();
      paramInputCheckbox
        .attr('tip-title', paramDesc.description)
        .prop('checked', !!paramVal)
        .on('change', updateParamBool)
        .show();
    } else if (paramDesc.type === 'int' || paramDesc.type === 'double') {
      paramInputNumber
        .attr('tip-title', paramDesc.description)
        .attr('step', paramDesc.type === 'int' ? 1 : 'any')
        .attr('min', _.get(paramDesc, 'min', ''))
        .attr('max', _.get(paramDesc, 'max', ''))
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    } else if (paramDesc.type === 'str') {
      paramInputText
        .attr('tip-title', paramDesc.description)
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    } else if (paramDesc.type === 'Station') {
      paramInputStation
        .attr('tip-title', paramDesc.description)
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    } else if (paramDesc.type === 'Register') {
      paramInputRegister
        .attr('tip-title', paramDesc.description)
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    } else if (paramDesc.type.startsWith('v')) {
      populateParamInputVariable(paramDesc.type);
      paramInputVariable
        .attr('tip-title', paramDesc.description)
        .val(paramVal)
        .on('change', updateParamVal)
        .show();
    }
  }

  function populateParamInputParam(type) {
    var params = viz.models.rawParams();
    var globalParams = viz.models.globalParams();
    var variables = viz.models.variables();

    paramInputParam.html('');
    for (let p of params) {
      if (p.type !== type && p.type !== 'v' + type) {
        continue;
      }
      let option = $('<option>');
      let ref = '${' + p.name + '}';
      option.val(ref).text(ref);
      paramInputParam.append(option);
    }
    for (let p of globalParams) {
      if (p.type !== type) {
        continue;
      }
      let option = $('<option>');
      let ref = '${' + p.name + '}g';
      option.val(ref).text(ref);
      paramInputParam.append(option);
    }
    for (let v of variables) {
      if (v.type !== type) {
        continue;
      }
      let option = $('<option>');
      let ref = '${' + v.name + '}v';
      option.val(ref).text(ref);
      paramInputParam.append(option);
    }
  }

  function populateParamInputVariable(type) {
    var variables = viz.models.variables();

    paramInputVariable.html('');
    for (let v of variables) {
      if ('v' + v.type !== type) {
        continue;
      }
      let option = $('<option>');
      option.val(v.name).text(v.name);
      paramInputVariable.append(option);
    }
  }

  function updateParamBool() {
    /* jshint validthis: true */
    viz.models.push('updateParam');
    var paramKey = _param_skillDesc.params[_param_idx].key;
    _param_action.params[paramKey] = $(this).is(':checked');
    viz.models.updateAction(_param_action);
  }

  function updateParamVal() {
    /* jshint validthis: true */
    var paramKey = _param_skillDesc.params[_param_idx].key;
    if (!this.validity.valid) {
      $(this).val(_param_action.params[paramKey]);
      return;
    }
    viz.models.push('updateParam');
    _param_action.params[paramKey] = $(this).val();
    viz.models.updateAction(_param_action);
  }

  function updateParamInherit() {
    /* jshint validthis: true */
    viz.models.push('updateParam');
    var paramKey = _param_skillDesc.params[_param_idx].key;
    var inherit = $(this).is(':checked');
    _param_action.params[paramKey] = inherit ? '${}' : undefined;
    setupParamInput();
    viz.models.updateAction(_param_action);
  }

  $viz.on('scene.activeObject', function () {
    var ob = viz.scene.activeObject.get();

    if (viz.scene.mode !== EditMode.POINTER) {
      actionProperties.hide();
      paramProperties.hide();
      linkProperties.hide();
      return;
    }

    if (ob.action_id !== null && ob.outcome_key) {
      let actions = viz.models.rawActions();
      if (ob.action_id === 0) {
        if (actions[0] !== null) {
          linkProperties.show();
        } else {
          linkProperties.hide();
        }
      } else if (actions[ob.action_id].outcomes[ob.outcome_key] !== null) {
        linkProperties.show();
      } else {
        linkProperties.hide();
      }
      actionProperties.hide();
    } else if (ob.action_id !== null) {
      actionProperties.show();
      linkProperties.hide();
    } else {
      actionProperties.hide();
      linkProperties.hide();
    }
    activateParamZeroOrHide();
  });

  actionProperties.hide();
  paramProperties.hide();
  linkProperties.hide();

  function onKeydown(event) {
    if (
      viz.scene.mode !== EditMode.POINTER ||
      !viz.scene.activeObject.isAction() ||
      markerModal.visible()
    ) {
      return;
    }
    if (event.keyCode === 0x09) {
      // Tab
      event.preventDefault();
      paramInputs.trigger('mouseout').trigger('blur'); // hide tooltip and trigger input change event.
      if (event.shiftKey) {
        // Shift+Tab
        activateParamPrev();
      } else {
        activateParamNext();
      }
    }
  }
  $(document.body).on('keydown', onKeydown);

  /* Group 6: Misc */
  var searchToolsInit = false;
  var searchToolsVisible = true;
  var searchTools = toolbar.find('.toolbar-search-tools');
  var searchFieldTarget = toolbar.find('.toolbar-search-field');
  var searchField = new Select({
    target: searchFieldTarget[0],
    props: {
      buttonClass: 'rounded rounded-r-none'
    }
  });
  var searchCount = toolbar.find('.toolbar-search-count');
  var searchResult = [];
  var curSearchIdx = 0;
  var searchSkipHide = false;

  toolbar.find('.toolbar-search-close').on('click', hideSearchTools);

  var searchPrev = toolbar.find('.toolbar-search-prev').on('click', function (e) {
    e.preventDefault();
    if (curSearchIdx <= 0) {
      return;
    }
    updateSearchCount(curSearchIdx - 1);
  });
  var searchNext = toolbar.find('.toolbar-search-next').on('click', function (e) {
    e.preventDefault();
    if (curSearchIdx >= searchResult.length - 1) {
      return;
    }
    updateSearchCount(curSearchIdx + 1);
  });

  searchField.$on('change', updateSearch);

  function updateSearch(value) {
    searchResult = viz.search.find(searchField.value || value) || [];
    updateSearchCount(0);
  }

  function updateSearchCount(count) {
    curSearchIdx = count;
    searchPrev.prop('disabled', count <= 0);
    searchNext.prop('disabled', count >= searchResult.length - 1);

    if (searchResult.length === 0) {
      searchCount.text('-/-');
    } else {
      searchCount.text(`${curSearchIdx + 1}/${searchResult.length}`);
      searchSkipHide = true;
      let cur = searchResult[curSearchIdx];
      viz.scene.activeObject.set(cur.action_id, cur.outcome_key, true);
    }
  }

  function showSearchTools(initSearch = '') {
    if (searchToolsVisible) {
      return;
    }
    searchToolsVisible = true;

    searchSkipHide = true;
    viz.scene.activeObject.set(null);
    searchTools.show();

    searchField.value = '';
    searchResult = [];
    updateSearchCount(0);

    initSearchTools();

    if (initSearch) {
      searchField.value = initSearch;
      updateSearch(initSearch);
    } else {
      searchField.show();
    }
  }

  function hideSearchTools() {
    if (!searchToolsVisible) {
      return;
    }
    searchToolsVisible = false;
    searchTools.hide();
  }

  $viz.on('models.ready', function () {
    let search = viz.models.rawSearch();
    if (!search) {
      return;
    }

    // Allow render graph first.
    window.setTimeout(function () {
      showSearchTools(search);
    }, 500);
  });

  $viz.on('scene.activeObject', function () {
    if (searchSkipHide) {
      searchSkipHide = false;
      return;
    }
    hideSearchTools();
  });

  function initSearchTools() {
    if (searchToolsInit) {
      return;
    }
    searchToolsInit = true;

    viz.search.fillOptions(searchField);
  }

  function onKeydownSearch(event) {
    if (event.ctrlKey && event.keyCode === 0x46) {
      // Ctrl + F
      showSearchTools();
      return false;
    }
  }
  $(document.body).on('keydown', onKeydownSearch);
  hideSearchTools();

  return {
    destroy() {
      $(document.body).off('keyup', onKeyupAO);
      $(document.body).off('keyup', onKeyupA1);
      $(document).off('cut', onCut);
      $(document).off('copy', onCopy);
      $(document).off('paste', onPaste);
      $(document.body).off('keyup', onKeyup);
      $(document.body).off('keydown', onKeydown);
      $(document.body).off('keydown', onKeydownSearch);
    }
  };
}
