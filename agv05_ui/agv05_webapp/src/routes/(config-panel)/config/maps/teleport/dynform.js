/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import taskTemplates from '$lib/shared/services/config/task-templates';
import { getEnv } from 'stores/auth';
import Select from 'components/Select.svelte';
import filterUtils from 'task-template-editor/search/utils';

var dynformHtml = `
<div class="table-container">
  <table class="table table-hover">
    <thead>
      <tr>
        <th colspan="7">Teleports</th>
      </tr>
      <tr>
        <th>#</th>
        <th style="width:12.5%; min-width:70px">Start</th>
        <th style="width:12.5%; min-width:70px">End</th>
        <th>Pre-Action</th>
        <th>Action</th>
        <th>Attributes</th>
        <th>Operation</th>
      </tr>
    </thead>
    <tbody>
    </tbody>
    <tfoot>
      <tr>
        <td colspan="7">
          <button type="button" class="variant-filled btn rounded py-1 form-add-teleport">
            <i class="fa-solid fa-plus mr-2"></i> Add teleport
          </button>
        </td>
      </tr>
    </tfoot>
  </table>
</div>
`;

var teleportInlineFormHtml = `
<tr class="teleport-inline">
  <td class="teleport-number"></td>
  <td><select class="select rounded teleport-input-start" required></select></td>
  <td><select class="select rounded teleport-input-end" required></select></td>
  <td class="space-y-3">
    <div class="flex items-center">
      <select class="select flex-1 rounded teleport-input-pre-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="space-y-1"></div>
  </td>
  <td class="space-y-3">
    <div class="flex items-center">
      <select class="select flex-1 rounded teleport-input-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="space-y-1"></div>
  </td>
  <td class="space-y-3">
    <div class="grid grid-cols-2 space-x-3">
      <span class="text-left m-auto w-full">Align Station Type (before teleport)</span>
      <select class="select rounded py-1 text-sm teleport-input-align-station-type" required>
        <option value="-1">Trackless</option>
        <option value="0">Front Line Sensor</option>
        <option value="1">Rear Line Sensor</option>
      </select>
    </div>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox teleport-input-non-stop-transition"/>
      <p>Non-stop Transition (before teleport)</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox teleport-input-auto-reset-agv-position"/>
      <div class="flex space-x-1 items-center">
        <p>Auto-reset AGV</p>
        <select class="select rounded py-1 text-sm flex-1 teleport-input-auto-reset-agv-position-tracker" required>
          <option value="1">Position</option>
          <option value="2">Position Tracker</option>
        </select>
        <p>(after teleport)</p>
      </div>
    </label>
    <label class="grid grid-cols-[auto_1fr] items-center space-x-2">
      <input type="checkbox" class="checkbox teleport-input-validate-rfid"/>
      <p>Validate RFID (after teleport)</p>
    </label>
    <div class="grid grid-cols-2 space-x-3 items-center">
      <span class="text-left">Distance Cost (m)</span>
      <input type="number" class="flex-1 input rounded py-1 text-sm teleport-input-distance" step="0.01" min="0.01" max="10000" required/>
    </div>
  </td>
  <td>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 teleport-move-up" tip-title="Move Up">
      <i class="fa fa-arrow-up"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 teleport-move-down" tip-title="Move Down">
      <i class="fa fa-arrow-down"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 teleport-clone" tip-title="Clone">
      <i class="fa fa-clone"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 teleport-remove" tip-title="Delete">
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
      <input type="checkbox" class="checkbox param-inherit my-1" tip-title="Inherit" tip-offset="9"/>
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

  var addTeleport = dynform
    .find('.form-add-teleport')
    .on('click', function () {
      var teleport = dynform.models.rawTeleports()[dynform.models.addTeleport()];
      addTeleportInlineForm(teleport);
      updateTeleportNumbering();
      clearTeleportFilter();
    })
    .prop('disabled', true);

  function addTeleportInlineForm(teleport) {
    var inlineForm = $(teleportInlineFormHtml);

    var inputStart = inlineForm
      .find('.teleport-input-start')
      .html(
        teleport.start && dynform.models.stationList().includes(teleport.start)
          ? optionHtml(teleport.start)
          : ''
      )
      .val(teleport.start)
      .on('mouseenter focus', () =>
        showParamDropdown(inputStart, prefabOptions.st, teleport.start)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputStart))
      .on('change', function () {
        teleport.start = inputStart.val();
        dynform.models.updateTeleport(teleport);
      });
    var inputEnd = inlineForm
      .find('.teleport-input-end')
      .html(
        teleport.end && dynform.models.stationList().includes(teleport.end)
          ? optionHtml(teleport.end)
          : ''
      )
      .val(teleport.end)
      .on('mouseenter focus', () =>
        showParamDropdown(inputEnd, prefabOptions.st, teleport.end)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputEnd))
      .on('change', function () {
        teleport.end = inputEnd.val();
        dynform.models.updateTeleport(teleport);
      });
    var inputPreAction = inlineForm
      .find('.teleport-input-pre-action')
      .html(prefabOptions.ac)
      .val(teleport.preAction.skillId)
      .on('change', function () {
        var _skillId = teleport.preAction.skillId;
        teleport.preAction.skillId = inputPreAction.val();
        setupLink(inputPreAction.next(), teleport.preAction);
        setupParamInputs(inputPreAction.parent().next(), teleport, teleport.preAction, {
          _skillId: _skillId || ''
        });
        dynform.models.updateTeleport(teleport);
      });
    var inputAction = inlineForm
      .find('.teleport-input-action')
      .html(prefabOptions.ac)
      .val(teleport.action.skillId)
      .on('change', function () {
        var _skillId = teleport.action.skillId;
        teleport.action.skillId = inputAction.val();
        setupLink(inputAction.next(), teleport.action);
        setupParamInputs(inputAction.parent().next(), teleport, teleport.action, {
          _skillId: _skillId || ''
        });
        dynform.models.updateTeleport(teleport);
      });
    var inputAlignStationType = inlineForm
      .find('.teleport-input-align-station-type')
      .val(teleport.alignStationType)
      .on('change', function () {
        teleport.alignStationType = inputAlignStationType.val();
        dynform.models.updateTeleport(teleport);
      });
    if (!getEnv('TRACKLESS')) {
      // remove "Trackless" option
      inputAlignStationType.children().first().remove();
    }
    var inputNonStopTransition = inlineForm
      .find('.teleport-input-non-stop-transition')
      .prop('checked', teleport.nonStopTransition)
      .on('change', function () {
        teleport.nonStopTransition = inputNonStopTransition.is(':checked');
        dynform.models.updateTeleport(teleport);
      });
    var inputAutoResetAgvPosition = inlineForm
      .find('.teleport-input-auto-reset-agv-position')
      .prop('checked', teleport.autoResetAgvPosition)
      .on('change', function () {
        teleport.autoResetAgvPosition = inputAutoResetAgvPosition.is(':checked')
          ? inputAutoResetAgvPositionTracker.val()
          : 0;
        inputAutoResetAgvPositionTracker.prop('disabled', !teleport.autoResetAgvPosition);
        dynform.models.updateTeleport(teleport);
      });
    var inputAutoResetAgvPositionTracker = inlineForm
      .find('.teleport-input-auto-reset-agv-position-tracker')
      .prop('disabled', !teleport.autoResetAgvPosition)
      .on('change', function () {
        if (inputAutoResetAgvPosition.is(':checked')) {
          teleport.autoResetAgvPosition = inputAutoResetAgvPositionTracker.val();
          dynform.models.updateTeleport(teleport);
        }
      });
    inputAutoResetAgvPositionTracker
      .find(`option[value="${teleport.autoResetAgvPosition}"]`) // ensure option exists
      .prop('selected', true);
    if (!getEnv('TRACKLESS')) {
      // remove "Position Tracker" option
      inputAutoResetAgvPositionTracker.children().last().remove();
      inputAutoResetAgvPositionTracker.before('<span>Position</span>');
      inputAutoResetAgvPositionTracker.remove();
    }
    var inputValidateRfid = inlineForm
      .find('.teleport-input-validate-rfid')
      .prop('checked', teleport.validateRfid)
      .on('change', function () {
        teleport.validateRfid = inputValidateRfid.is(':checked');
        dynform.models.updateTeleport(teleport);
      });
    inputValidateRfid.closest('label').toggle(!getEnv('TRACKLESS'));
    var inputDistance = inlineForm
      .find('.teleport-input-distance')
      .val(teleport.distance)
      .on('change', function () {
        teleport.distance = inputDistance.val();
        dynform.models.updateTeleport(teleport);
      });
    setupLink(inputPreAction.next(), teleport.preAction);
    setupParamInputs(inputPreAction.parent().next(), teleport, teleport.preAction);
    setupLink(inputAction.next(), teleport.action);
    setupParamInputs(inputAction.parent().next(), teleport, teleport.action);

    inlineForm.find('.teleport-move-up').on('click', function () {
      if (dynform.models.moveUpTeleport(teleport)) {
        inlineForm.insertBefore(inlineForm.prev());
        updateTeleportNumbering();
      }
    });
    inlineForm.find('.teleport-move-down').on('click', function () {
      if (dynform.models.moveDownTeleport(teleport)) {
        inlineForm.insertAfter(inlineForm.next());
        updateTeleportNumbering();
      }
    });
    inlineForm.find('.teleport-clone').on('click', function () {
      var inlineFormClone = addTeleportInlineForm(
        dynform.models.rawTeleports()[dynform.models.cloneTeleport(teleport)]
      );
      inlineFormClone.insertAfter(inlineForm);
      updateTeleportNumbering();
    });
    inlineForm.find('.teleport-remove').on('click', function () {
      if (dynform.models.removeTeleport(teleport)) {
        inlineForm.remove();
        updateTeleportNumbering();
      }
    });

    tbody.append(inlineForm);
    return inlineForm;
  }

  function setupLink(link, action) {
    if (action.skillId && action.skillId.indexOf('_ttpk_') === 0) {
      link.attr('href', taskTemplates.editUrl(action.skillId.slice(6))).show();
    } else {
      link.removeAttr('href').hide();
    }
  }

  function setupParamInputs(outer, teleport, action, reset) {
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
        dynform.models.updateTeleport(teleport);
      }

      function _updateParamBool() {
        /* jshint validthis: true */
        action.params[paramDesc.key] = $(this).is(':checked');
        dynform.models.updateTeleport(teleport);
      }

      function _updateParamInherit() {
        var inherit = paramInherit.is(':checked');
        action.params[paramDesc.key] = inherit ? '${}' : undefined;
        _updateParamInput();
        dynform.models.updateTeleport(teleport);
      }
    });

    if (!skillDesc.params.length) {
      outer.text('-');
    }
  }

  function updateTeleportNumbering() {
    tbody.find('.teleport-number').each(function (idx, elem) {
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
      updateTeleportFilter();
    });

    filterList.append(btn);
    updateTeleportFilter();
  }

  function clearTeleportFilter() {
    filterList.empty();
    filters = [];
    updateTeleportFilter();
  }

  function updateTeleportFilter() {
    let row = tbody.children('tr');
    let teleports = dynform.models.rawTeleports();

    for (let idx in teleports) {
      let r = row.eq(idx);
      r.show();
      let teleport = teleports[idx];
      for (let id of filters) {
        if (!findInTeleport(teleport, id)) {
          r.hide();
          break;
        }
      }
    }
  }

  function findInTeleport(teleport, id) {
    if (id.startsWith('_Station_')) {
      return findStationInTeleport(teleport, id);
    } else if (id.startsWith('_Register_')) {
      return findRegisterInTeleport(teleport, id);
    } else if (id.startsWith('_g_')) {
      return findGlobalParamInTeleport(teleport, id);
    } else if (id.startsWith('_v_')) {
      return findVariableInTeleport(teleport, id);
    } else if (id.startsWith('_ttpk_')) {
      return findTaskTemplateInTeleport(teleport, id);
    } else if (id.startsWith('_a_')) {
      return findTaskTemplateInTeleport(teleport, id.substring(3));
    }
  }

  function findStationInTeleport(teleport, id) {
    let station = id.substr(9);

    if (teleport.start === station || teleport.end === station) {
      return true;
    }

    if (filterUtils.findStationInAction(teleport.preAction, station, dynform.models)) {
      return true;
    }
    if (filterUtils.findStationInAction(teleport.action, station, dynform.models)) {
      return true;
    }
  }

  function findRegisterInTeleport(teleport, id) {
    let register = id.substr(10);

    if (filterUtils.findRegisterInAction(teleport.preAction, register, dynform.models)) {
      return true;
    }
    if (filterUtils.findRegisterInAction(teleport.action, register, dynform.models)) {
      return true;
    }
  }

  function findGlobalParamInTeleport(teleport, id) {
    let gp = id.substr(3);
    gp = '${' + gp + '}g';

    if (filterUtils.findGlobalParamInAction(teleport.preAction, gp, dynform.models)) {
      return true;
    }
    if (filterUtils.findGlobalParamInAction(teleport.action, gp, dynform.models)) {
      return true;
    }
  }

  function findVariableInTeleport(teleport, id) {
    let variable = id.substr(3);

    if (filterUtils.findVariableInAction(teleport.preAction, variable, dynform.models)) {
      return true;
    }
    if (filterUtils.findVariableInAction(teleport.action, variable, dynform.models)) {
      return true;
    }
  }

  function findTaskTemplateInTeleport(teleport, id) {
    if (teleport.preAction.skillId === id) {
      return true;
    }
    if (teleport.action.skillId === id) {
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

    var teleportParams = {
      int: ['auto_reset_agv_position', 'next_motion', 'pre_steps'],
      double: ['distance_cost'],
      Station: ['start', 'end']
    };
    prefabOptions.params = {};
    for (let k in teleportParams) {
      prefabOptions.params[k] = '';
      for (let n of teleportParams[k]) {
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

    var teleports = dynform.models.rawTeleports();
    for (let teleport of teleports) {
      addTeleportInlineForm(teleport);
    }
    updateTeleportNumbering();
    addTeleport.prop('disabled', false);

    filterUtils.fillOptions(filterField, dynform.models);
    let f = dynform.models.rawFilter();
    if (f) {
      filterField.value = f;
      addFilter(f);
    }
  });

  dynform.on('models.dirty', function () {
    updateTeleportFilter();
  });

  return dynform;
}
