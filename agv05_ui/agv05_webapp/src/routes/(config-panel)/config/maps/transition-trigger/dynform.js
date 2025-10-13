/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import taskTemplates from '$lib/shared/services/config/task-templates';
import filterUtils from 'task-template-editor/search/utils';
import Select from 'components/Select.svelte';

var dynformHtml = `
<div class="table-container">
  <table class="table table-hover">
    <thead>
      <tr>
        <th colspan="7">Transition Triggers</th>
      </tr>
      <tr>
        <th>#</th>
        <th style="width:12.5%; min-width:70px">Start</th>
        <th style="width:12.5%; min-width:70px">End</th>
        <th>Start Action (before rotate align)</th>
        <th>End Action (after rotate align)</th>
        <th>Attributes</th>
        <th>Operation</th>
      </tr>
    </thead>
    <tbody>
    </tbody>
    <tfoot>
      <tr>
        <td colspan="7">
          <button type="button" class="variant-filled btn rounded py-1 form-add-transition-trigger">
            <i class="fa-solid fa-plus mr-2"></i> Add transition trigger
          </button>
        </td>
      </tr>
    </tfoot>
  </table>
</div>
`;

var transitionTriggerInlineFormHtml = `
<tr class="transition-trigger-inline">
  <td class="transition-trigger-number"></td>
  <td><select class="select rounded transition-trigger-input-start" required></select></td>
  <td><select class="select rounded transition-trigger-input-end" required></select></td>
  <td class="space-y-3">
    <div class="flex items-center">
      <select class="select flex-1 rounded transition-trigger-input-start-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="space-y-1"></div>
  </td>
  <td class="space-y-3">
    <div class="flex items-center">
      <select class="select flex-1 rounded transition-trigger-input-end-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="space-y-1"></div>
  </td>
  <td class="space-y-3">
    <div class="grid grid-cols-2 space-x-3">
      <span class="text-left m-auto w-full" tip-title="Select the motion(s) that should trigger the action">Applicable Motion (after transition)</span>
      <div class="transition-trigger-input-applicable-motion-target flex flex-row-reverse"></div>
    </div>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox transition-trigger-input-cancel-non-stop-transition"/>
      <p>Always stop (before transition)</p>
    </label>
  </td>
  <td>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 transition-trigger-move-up" tip-title="Move Up">
      <i class="fa fa-arrow-up"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 transition-trigger-move-down" tip-title="Move Down">
      <i class="fa fa-arrow-down"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 transition-trigger-clone" tip-title="Clone">
      <i class="fa fa-clone"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 transition-trigger-remove" tip-title="Delete">
      <i class="fa fa-times"></i>
    </button>
  </td>
</tr>
`;

var paramHeaderHtml = `
<label class="text-base font-semibold text-left">Parameters:</label>
`;

var paramRowHtml = `
<div class="flex space-x-3">
  <span class="param-label m-auto"></span>
  <div class="flex-1 rounded grid grid-cols-[1fr_auto] border border-surface-400-500-token bg-surface-200-700-token">
    <input class="param-input input py-1 rounded rounded-r-none text-sm border-0"/>
    <input type="checkbox" class="param-input checkbox m-auto p-3"/>
    <select class="param-input select py-1 rounded rounded-r-none text-sm border-0"></select>
    <div class="px-3 bg-surface-300-600-token h-full rounded rounded-l-none border-l border-surface-400-500-token">
      <input type="checkbox" class="checkbox param-inherit my-1"/>
    </div>
  </div>
</div>
`;

var optionHtml = (val) => `<option value="${val}">${val}</option>`;

var filterHtml = `
<div class="filter-container">
  <div class="inline-flex">
    <div class="filter-field inline-flex"></div>
    <button type="button" class="filter-button btn rounded variant-filled rounded-l-none border-l border-gray-700 dark:border-gray-300">
      Add Filter
    </button>
  </div>
  <div class="filter-list col-sm-6">
  </div>
</div>
<div class="clearfix">&nbsp;</div>
`;

var filterBtnHtml = (text) => `
<a class="btn variant-filled rounded mt-1">
  ${text}
  <i class="fa fa-times pl-1"></i>
</a>
`;

export default function (editor) {
  var dynform = $(dynformHtml);
  var tbody = dynform.find('tbody');
  var prefabOptions = {};

  editor.append(dynform);

  var addTransitionTrigger = dynform
    .find('.form-add-transition-trigger')
    .on('click', function () {
      var transitionTrigger =
        dynform.models.rawTransitionTriggers()[dynform.models.addTransitionTrigger()];
      addTransitionTriggerInlineForm(transitionTrigger);
      updateTransitionTriggerNumbering();
      clearTransitionTriggerFilter();
    })
    .prop('disabled', true);

  function addTransitionTriggerInlineForm(transitionTrigger) {
    var inlineForm = $(transitionTriggerInlineFormHtml);
    // NOTE: Svelte Select component required dom to be mount first.
    tbody.append(inlineForm);

    var inputStart = inlineForm
      .find('.transition-trigger-input-start')
      .html(
        transitionTrigger.start &&
          dynform.models.stationList().includes(transitionTrigger.start)
          ? optionHtml(transitionTrigger.start)
          : ''
      )
      .val(transitionTrigger.start)
      .on('mouseenter focus', () =>
        showParamDropdown(inputStart, prefabOptions.st, transitionTrigger.start)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputStart))
      .on('change', function () {
        transitionTrigger.start = inputStart.val();
        dynform.models.updateTransitionTrigger(transitionTrigger);
      });
    var inputEnd = inlineForm
      .find('.transition-trigger-input-end')
      .html(
        transitionTrigger.end && dynform.models.stationList().includes(transitionTrigger.end)
          ? optionHtml(transitionTrigger.end)
          : ''
      )
      .val(transitionTrigger.end)
      .on('mouseenter focus', () =>
        showParamDropdown(inputEnd, prefabOptions.st, transitionTrigger.end)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputEnd))
      .on('change', function () {
        transitionTrigger.end = inputEnd.val();
        dynform.models.updateTransitionTrigger(transitionTrigger);
      });
    var inputStartAction = inlineForm
      .find('.transition-trigger-input-start-action')
      .html(prefabOptions.ac)
      .val(transitionTrigger.startAction.skillId)
      .on('change', function () {
        var _skillId = transitionTrigger.startAction.skillId;
        transitionTrigger.startAction.skillId = inputStartAction.val();
        setupLink(inputStartAction.next(), transitionTrigger.startAction);
        setupParamInputs(
          inputStartAction.parent().next(),
          transitionTrigger,
          transitionTrigger.startAction,
          {
            _skillId: _skillId || ''
          }
        );
        dynform.models.updateTransitionTrigger(transitionTrigger);
      });
    var inputEndAction = inlineForm
      .find('.transition-trigger-input-end-action')
      .html(prefabOptions.ac)
      .val(transitionTrigger.endAction.skillId)
      .on('change', function () {
        var _skillId = transitionTrigger.endAction.skillId;
        transitionTrigger.endAction.skillId = inputEndAction.val();
        setupLink(inputEndAction.next(), transitionTrigger.endAction);
        setupParamInputs(
          inputEndAction.parent().next(),
          transitionTrigger,
          transitionTrigger.endAction,
          {
            _skillId: _skillId || ''
          }
        );
        dynform.models.updateTransitionTrigger(transitionTrigger);
      });
    var inputApplicableMotionTarget = inlineForm.find(
      '.transition-trigger-input-applicable-motion-target'
    );
    var inputApplicableMotion = new Select({
      target: inputApplicableMotionTarget[0],
      props: {
        buttonClass: 'rounded py-1 text-sm variant-form h-7',
        class: 'w-[250px]',
        enableFilter: false,
        startEmpty: true,
        multiple: true,
        value: transitionTrigger.applicableMotion,
        options: [
          [0, 'Forward departure'],
          [1, 'Reverse departure'],
          [2, 'Forward intermediate'],
          [3, 'Reverse intermediate'],
          [4, 'Rotate align at destination']
        ]
      }
    });
    inputApplicableMotion.$on('change', function () {
      transitionTrigger.applicableMotion = inputApplicableMotion.value;
      dynform.models.updateTransitionTrigger(transitionTrigger);
    });

    var inputCancleNonStopTransitionTrigger = inlineForm
      .find('.transition-trigger-input-cancel-non-stop-transition')
      .prop('checked', transitionTrigger.cancelNonStopTransition)
      .on('change', function () {
        transitionTrigger.cancelNonStopTransition =
          inputCancleNonStopTransitionTrigger.is(':checked');
        dynform.models.updateTransitionTrigger(transitionTrigger);
      });
    setupLink(inputStartAction.next(), transitionTrigger.startAction);
    setupParamInputs(
      inputStartAction.parent().next(),
      transitionTrigger,
      transitionTrigger.startAction
    );
    setupLink(inputEndAction.next(), transitionTrigger.endAction);
    setupParamInputs(
      inputEndAction.parent().next(),
      transitionTrigger,
      transitionTrigger.endAction
    );

    inlineForm.find('.transition-trigger-move-up').on('click', function () {
      if (dynform.models.moveUpTransitionTrigger(transitionTrigger)) {
        inlineForm.insertBefore(inlineForm.prev());
        updateTransitionTriggerNumbering();
      }
    });
    inlineForm.find('.transition-trigger-move-down').on('click', function () {
      if (dynform.models.moveDownTransitionTrigger(transitionTrigger)) {
        inlineForm.insertAfter(inlineForm.next());
        updateTransitionTriggerNumbering();
      }
    });
    inlineForm.find('.transition-trigger-clone').on('click', function () {
      var inlineFormClone = addTransitionTriggerInlineForm(
        dynform.models.rawTransitionTriggers()[
          dynform.models.cloneTransitionTrigger(transitionTrigger)
        ]
      );
      inlineFormClone.insertAfter(inlineForm);
      updateTransitionTriggerNumbering();
    });
    inlineForm.find('.transition-trigger-remove').on('click', function () {
      if (dynform.models.removeTransitionTrigger(transitionTrigger)) {
        inlineForm.remove();
        updateTransitionTriggerNumbering();
      }
    });

    return inlineForm;
  }

  function setupLink(link, action) {
    if (action.skillId && action.skillId.indexOf('_ttpk_') === 0) {
      link.attr('href', taskTemplates.editUrl(action.skillId.slice(6))).show();
    } else {
      link.removeAttr('href').hide();
    }
  }

  function setupParamInputs(outer, transitionTrigger, action, reset) {
    outer.empty();
    if (!action.skillId) {
      return;
    }

    var skillDesc;
    var formatLabel;
    if (action.skillId.indexOf('_ttpk_') === 0) {
      skillDesc = dynform.models.taskTemplateMetas()[action.skillId];
      formatLabel = _.identity;
    } else {
      skillDesc = dynform.models.skillDescriptions()[action.skillId];
      formatLabel = _.startCase;
    }
    if (!skillDesc || !skillDesc.params) {
      return;
    }

    if (reset && action.skillId !== reset._skillId) {
      var _skillDesc;
      if (reset._skillId.indexOf('_ttpk_') === 0) {
        _skillDesc = dynform.models.taskTemplateMetas()[reset._skillId];
      } else {
        _skillDesc = dynform.models.skillDescriptions()[reset._skillId];
      }
      var _descParams = (_skillDesc && _skillDesc.params) || [];
      var _params = action.params;

      action.params = {};
      for (let p of skillDesc.params) {
        if (p.type === 'bool') {
          action.params[p.key] = typeof p.default === typeof false ? p.default : false;
        } else {
          action.params[p.key] = p.default;
        }
        for (let p2 of _descParams) {
          if (p.name === p2.name) {
            if (p.type === p2.type && p2.key in _params) {
              action.params[p.key] = _params[p2.key];
            }
            break;
          }
        }
      }
    }

    outer.append(paramHeaderHtml);
    skillDesc.params.forEach(function (paramDesc) {
      var paramRow = $(paramRowHtml);
      paramRow.find('.param-label').html(formatLabel(paramDesc.name));
      var paramInputs = paramRow.find('.param-input').attr('tip-title', paramDesc.description);
      var paramInherit = paramRow.find('.param-inherit').on('change', _updateParamInherit);

      _updateParamInput();
      outer.append(paramRow);

      function _updateParamInput() {
        var paramVal = action.params[paramDesc.key];
        paramInputs
          .prop('required', false)
          .hide()
          .off('change focus blur mouseenter mouseleave');
        paramInherit.prop('checked', false);

        if (_.isString(paramVal) && paramVal.indexOf('${') === 0) {
          paramInherit.prop('checked', true);
          paramInputs
            .last()
            .html(prefabOptions.params[paramDesc.type] || '')
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type === 'bool') {
          paramInputs
            .filter('.checkbox')
            .prop('checked', !!paramVal)
            .on('change', _updateParamBool)
            .show();
        } else if (paramDesc.type === 'int' || paramDesc.type === 'double') {
          paramInputs
            .first()
            .attr({
              type: 'number',
              step: paramDesc.type === 'int' ? 1 : 'any',
              min: paramDesc.min,
              max: paramDesc.max
            })
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type === 'str') {
          paramInputs
            .first()
            .attr('type', 'text')
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type === 'Station') {
          if (paramVal && dynform.models.stationList().includes(paramVal)) {
            paramInputs.last().html(optionHtml(paramVal)).val(paramVal);
          } else {
            paramInputs.last().empty().val('');
          }
          paramInputs
            .last()
            .prop('required', true)
            .on('mouseenter focus', () =>
              showParamDropdown(
                paramInputs.last(),
                prefabOptions.st,
                action.params[paramDesc.key]
              )
            )
            .on('mouseleave blur', () => hideParamDropdown(paramInputs.last()))
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type === 'Register') {
          if (paramVal && dynform.models.registerList().includes(paramVal)) {
            paramInputs.last().html(optionHtml(paramVal)).val(paramVal);
          } else {
            paramInputs.last().empty().val('');
          }
          paramInputs
            .last()
            .prop('required', true)
            .on('mouseenter focus', () =>
              showParamDropdown(
                paramInputs.last(),
                prefabOptions.reg,
                action.params[paramDesc.key]
              )
            )
            .on('mouseleave blur', () => hideParamDropdown(paramInputs.last()))
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type.startsWith('v')) {
          paramInputs
            .last()
            .html(prefabOptions.variables[paramDesc.type] || '')
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        }
      }

      function _updateParamVal() {
        /* jshint validthis: true */
        action.params[paramDesc.key] = $(this).val();
        dynform.models.updateTransitionTrigger(transitionTrigger);
      }

      function _updateParamBool() {
        /* jshint validthis: true */
        action.params[paramDesc.key] = $(this).is(':checked');
        dynform.models.updateTransitionTrigger(transitionTrigger);
      }

      function _updateParamInherit() {
        var inherit = paramInherit.is(':checked');
        action.params[paramDesc.key] = inherit ? '${}' : undefined;
        _updateParamInput();
        dynform.models.updateTransitionTrigger(transitionTrigger);
      }
    });

    if (!skillDesc.params.length) {
      outer.text('-');
    }
  }

  function updateTransitionTriggerNumbering() {
    tbody.find('.transition-trigger-number').each(function (idx, elem) {
      $(elem).text(idx + 1);
    });
  }

  function showParamDropdown(input, options, val) {
    /* populate dropdown options */
    if (input[0].childElementCount <= 1) {
      input.html(options).val(val);
    }
  }

  function hideParamDropdown(input, empty = '') {
    /* remove dropdown options */
    if (input.is(':focus')) {
      return;
    }
    var v = input.val();
    if (v) {
      input.html(optionHtml(v)).val(v);
    } else {
      input.html(empty).val('');
    }
  }

  // Filter Section
  var filter = $(filterHtml);
  dynform.before(filter);
  var filterFieldTarget = filter.find('.filter-field');
  var filterField = new Select({
    target: filterFieldTarget[0],
    props: {
      buttonClass: 'rounded rounded-r-none'
    }
  });
  var filterList = filter.find('.filter-list');
  var filters = [];

  filter.find('.filter-button').on('click', function (e) {
    e.preventDefault();
    addFilter();
  });

  function addFilter(value) {
    let val = filterField.value || value;
    if (!val || filters.indexOf(val) >= 0) {
      return;
    }
    filters.push(val);

    let text = filterField.getDisplayVal() || value;
    let btn = $(filterBtnHtml(text));

    btn.on('click', function () {
      let idx = filters.indexOf(val);
      if (idx < 0) {
        return;
      }
      btn.remove();
      filters.splice(idx, 1);
      updateTransitionTriggerFilter();
    });

    filterList.append(btn);
    updateTransitionTriggerFilter();
  }

  function clearTransitionTriggerFilter() {
    filterList.empty();
    filters = [];
    updateTransitionTriggerFilter();
  }

  function updateTransitionTriggerFilter() {
    let row = tbody.children('tr');
    let transitionTriggers = dynform.models.rawTransitionTriggers();

    for (let idx in transitionTriggers) {
      let r = row.eq(idx);
      r.show();
      let transitionTrigger = transitionTriggers[idx];
      for (let id of filters) {
        if (!findInTransitionTrigger(transitionTrigger, id)) {
          r.hide();
          break;
        }
      }
    }
  }

  function findInTransitionTrigger(transitionTrigger, id) {
    if (id.startsWith('_Station_')) {
      return findStationInTransitionTrigger(transitionTrigger, id);
    } else if (id.startsWith('_Register_')) {
      return findRegisterInTransitionTrigger(transitionTrigger, id);
    } else if (id.startsWith('_g_')) {
      return findGlobalParamInTransitionTrigger(transitionTrigger, id);
    } else if (id.startsWith('_v_')) {
      return findVariableInTransitionTrigger(transitionTrigger, id);
    } else if (id.startsWith('_ttpk_')) {
      return findTaskTemplateInTransitionTrigger(transitionTrigger, id);
    } else if (id.startsWith('_a_')) {
      return findTaskTemplateInTransitionTrigger(transitionTrigger, id.substring(3));
    }
  }

  function findStationInTransitionTrigger(transitionTrigger, id) {
    let station = id.substr(9);

    if (transitionTrigger.start === station || transitionTrigger.end === station) {
      return true;
    }

    if (
      filterUtils.findStationInAction(transitionTrigger.startAction, station, dynform.models)
    ) {
      return true;
    }
    if (
      filterUtils.findStationInAction(transitionTrigger.endAction, station, dynform.models)
    ) {
      return true;
    }
  }

  function findRegisterInTransitionTrigger(transitionTrigger, id) {
    let register = id.substr(10);

    if (
      filterUtils.findRegisterInAction(transitionTrigger.startAction, register, dynform.models)
    ) {
      return true;
    }
    if (
      filterUtils.findRegisterInAction(transitionTrigger.endAction, register, dynform.models)
    ) {
      return true;
    }
  }

  function findGlobalParamInTransitionTrigger(transitionTrigger, id) {
    let gp = id.substr(3);
    gp = '${' + gp + '}g';

    if (
      filterUtils.findGlobalParamInAction(transitionTrigger.startAction, gp, dynform.models)
    ) {
      return true;
    }
    if (filterUtils.findGlobalParamInAction(transitionTrigger.endAction, gp, dynform.models)) {
      return true;
    }
  }

  function findVariableInTransitionTrigger(transitionTrigger, id) {
    let variable = id.substr(3);

    if (
      filterUtils.findVariableInAction(transitionTrigger.startAction, variable, dynform.models)
    ) {
      return true;
    }
    if (
      filterUtils.findVariableInAction(transitionTrigger.endAction, variable, dynform.models)
    ) {
      return true;
    }
  }

  function findTaskTemplateInTransitionTrigger(transitionTrigger, id) {
    if (transitionTrigger.startAction.skillId === id) {
      return true;
    }
    if (transitionTrigger.endAction.skillId === id) {
      return true;
    }
  }

  dynform.on('models.ready', function () {
    var skillDescriptions = dynform.models.skillDescriptions();
    var taskTemplateMetas = dynform.models.taskTemplateMetas();
    prefabOptions.ac = $('<select>');
    prefabOptions.ac.append('<option value="">----</option>');
    var og = $('<optgroup label="Built-in">');
    for (let k in skillDescriptions) {
      let d = skillDescriptions[k];
      let option = $('<option>');
      option.val(d.id).text(d.name);
      og.append(option);
    }
    prefabOptions.ac.append(og);

    var og1 = $('<optgroup label="Non Top-level">');
    var og2 = $('<optgroup label="Top-level">');
    for (let k in taskTemplateMetas) {
      let d = taskTemplateMetas[k];
      let option = $('<option>');
      option.val(d.id).text(d.name);
      if (!d.is_active) {
        option.prop('disabled', true);
      }
      if (!d.is_top_level) {
        og1.append(option);
      } else {
        og2.append(option);
      }
    }
    prefabOptions.ac.append(og1, og2);
    prefabOptions.ac = prefabOptions.ac.html();

    var stationList = dynform.models.stationList();
    prefabOptions.st = '';
    for (let s of stationList) {
      prefabOptions.st += optionHtml(s);
    }

    var registerList = dynform.models.registerList();
    prefabOptions.reg = '';
    for (let r of registerList) {
      prefabOptions.reg += optionHtml(r);
    }

    var transitionTriggerParams = {
      int: ['next_motion'],
      Station: ['start', 'end']
    };
    prefabOptions.params = {};
    for (let k in transitionTriggerParams) {
      prefabOptions.params[k] = '';
      for (let n of transitionTriggerParams[k]) {
        let ref = '${' + n + '}';
        prefabOptions.params[k] += optionHtml(ref);
      }
    }

    var globalParams = dynform.models.globalParams();
    for (let gp of globalParams) {
      if (!prefabOptions.params[gp.type]) {
        prefabOptions.params[gp.type] = '';
      }
      prefabOptions.params[gp.type] += optionHtml('${' + gp.name + '}g');
    }

    var variableObjectList = dynform.models.variables();
    prefabOptions.variables = {};
    for (let v of variableObjectList) {
      let vType = 'v' + v.type;
      if (!prefabOptions.variables[vType]) {
        prefabOptions.variables[vType] = '';
      }
      prefabOptions.variables[vType] += optionHtml(v.name);

      if (!prefabOptions.params[v.type]) {
        prefabOptions.params[v.type] = '';
      }
      prefabOptions.params[v.type] += optionHtml('${' + v.name + '}v');
    }

    var transitionTriggers = dynform.models.rawTransitionTriggers();
    for (let transitionTrigger of transitionTriggers) {
      addTransitionTriggerInlineForm(transitionTrigger);
    }
    updateTransitionTriggerNumbering();
    addTransitionTrigger.prop('disabled', false);

    filterUtils.fillOptions(filterField, dynform.models);
    let f = dynform.models.rawFilter();
    if (f) {
      filterField.value = f;
      addFilter(f);
    }
  });

  dynform.on('models.dirty', function () {
    updateTransitionTriggerFilter();
  });

  return dynform;
}
