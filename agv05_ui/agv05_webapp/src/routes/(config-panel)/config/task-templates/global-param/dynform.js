/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import Select from 'components/Select.svelte';
import filterUtils from 'task-template-editor/search/utils';

var dynformHtml = `
<div class="table-container">
  <table class="table table-hover">
    <thead>
      <tr>
        <th colspan="6">Global Parameters</th>
      </tr>
      <tr>
        <th>#</th>
        <th>Name</th>
        <th>Type</th>
        <th>Description</th>
        <th style="width:30%">Value</th>
        <th>Operation</th>
      </tr>
    </thead>
    <tbody>
    </tbody>
    <tfoot>
      <tr>
        <td colspan="6">
          <button type="button" class="variant-filled btn rounded py-1 add-global-param">
            <i class="fa-solid fa-plus mr-2"></i> Add global parameter
          </button>
        </td>
      </tr>
    </tfoot>
  </table>
</div>
`;

var paramInlineFormHtml = `
<tr class="param-inline">
  <td class="param-number"></td>
  <td><input type="text" class="input rounded param-input-name" required/></td>
  <td><select class="select rounded param-input-type" required>
    <option value="bool">Bool</option>
    <option value="int" selected>Int</option>
    <option value="double">Double</option>
    <option value="str">Str</option>
    <option value="Station">Station</option>
    <option value="Register">Register</option>
  </select></td>
  <td><input type="text" class="input rounded param-input-description"/></td>
  <td class="text-center">
    <input type="checkbox" class="checkbox param-input param-input-checkbox w-10 h-10"/>
    <input type="text" class="input rounded param-input param-input-text" required/>
    <input type="number" class="input rounded param-input param-input-number" required/>
    <select class="select rounded param-input param-input-station" required></select>
    <select class="select rounded param-input param-input-register" required></select>
  </td>
  <td>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 param-move-up" tip-title="Move Up">
      <i class="fa fa-arrow-up"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 param-move-down" tip-title="Move Down">
      <i class="fa fa-arrow-down"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 param-remove" tip-title="Delete">
      <i class="fa fa-times"></i>
    </button>
  </td>
</tr>
`;

var optionHtml = (val) => `<option>${val}</option>`;

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

  var addGlobalParam = dynform
    .find('.add-global-param')
    .on('click', function () {
      var param =
        dynform.models.rawGlobalParams()[
          dynform.models.addGlobalParam({
            type: 'int'
          })
        ];
      addParamInlineForm(param);
      updateParamNumbering();
      clearGlobalParamFilter();
    })
    .prop('disabled', true);

  function addParamInlineForm(param) {
    var inlineForm = $(paramInlineFormHtml);

    var inputName = inlineForm
      .find('.param-input-name')
      .val(param.name)
      .on('change', function () {
        param.name = inputName.val();
        dynform.models.updateGlobalParam(param);
      });
    var inputType = inlineForm
      .find('.param-input-type')
      .val(param.type)
      .on('change', function () {
        param.type = inputType.val();
        param.default = param.type === 'bool' ? false : undefined;
        setupParamInput();
        dynform.models.updateGlobalParam(param);
      });
    var inputDesc = inlineForm
      .find('.param-input-description')
      .val(param.description)
      .on('change', function () {
        param.description = inputDesc.val();
        dynform.models.updateGlobalParam(param);
      });

    var inputs = inlineForm.find('.param-input');
    var inputCheckbox = inlineForm.find('.param-input-checkbox').on('change', updateParamBool);
    var inputNumber = inlineForm.find('.param-input-number').on('change', updateParamVal);
    var inputText = inlineForm.find('.param-input-text').on('change', updateParamVal);
    var inputStation = inlineForm
      .find('.param-input-station')
      .on('mouseenter focus', () =>
        showParamDropdown(inputStation, prefabOptions.st, param.default)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputStation))
      .on('change', updateParamVal);
    var inputRegister = inlineForm
      .find('.param-input-register')
      .on('mouseenter focus', () =>
        showParamDropdown(inputRegister, prefabOptions.reg, param.default)
      )
      .on('mouseleave blur', () => hideParamDropdown(inputRegister))
      .on('change', updateParamVal);
    setupParamInput();

    inlineForm.find('.param-move-up').on('click', function () {
      if (dynform.models.moveUpGlobalParam(param)) {
        inlineForm.insertBefore(inlineForm.prev());
        updateParamNumbering();
      }
    });
    inlineForm.find('.param-move-down').on('click', function () {
      if (dynform.models.moveDownGlobalParam(param)) {
        inlineForm.insertAfter(inlineForm.next());
        updateParamNumbering();
      }
    });
    inlineForm.find('.param-remove').on('click', function () {
      if (dynform.models.removeGlobalParam(param)) {
        inlineForm.remove();
        updateParamNumbering();
      }
    });

    tbody.append(inlineForm);

    function setupParamInput() {
      inputs.hide().prop('disabled', true);

      var val = param.default;
      switch (param.type) {
        case 'bool':
          inputCheckbox.prop('checked', !!val).prop('disabled', false).show();
          break;
        case 'int':
          inputNumber.attr('step', 1).val(val).prop('disabled', false).show();
          break;
        case 'double':
          inputNumber.attr('step', 'any').val(val).prop('disabled', false).show();
          break;
        case 'str':
          inputText.val(val).prop('disabled', false).show();
          break;
        case 'Station':
          if (val && dynform.models.stationList().includes(val)) {
            inputStation.html(optionHtml(val)).val(val);
          } else {
            inputStation.empty().val('');
          }
          inputStation.prop('disabled', false).show();
          break;
        case 'Register':
          if (val && dynform.models.registerList().includes(val)) {
            inputRegister.html(optionHtml(val)).val(val);
          } else {
            inputRegister.empty().val('');
          }
          inputRegister.prop('disabled', false).show();
          break;
      }
    }

    function updateParamBool() {
      /* jshint validthis: true */
      param.default = $(this).is(':checked');
      dynform.models.updateGlobalParam(param);
    }

    function updateParamVal() {
      /* jshint validthis: true */
      param.default = $(this).val();
      dynform.models.updateGlobalParam(param);
    }
  }

  function addReservedParamInlineForm(param) {
    var inlineForm = $(paramInlineFormHtml);

    inlineForm.find('.param-input-name').val(param.name).prop('disabled', true);
    inlineForm.find('.param-input-type').val(param.type).prop('disabled', true);
    inlineForm.find('.param-input-description').val(param.description).prop('disabled', true);
    inlineForm.find('.param-input').prop('disabled', true).hide();
    setupParamInput();
    inlineForm
      .children('td:last-child')
      .html('<div class="text-opacity-60 text-gray-500 select-none">(Reserved)</div>');

    tbody.append(inlineForm);

    function setupParamInput() {
      if (param.type === 'bool') {
        inlineForm.find('.param-input-checkbox').prop('checked', !!param.default).show();
      } else {
        inlineForm.find('.param-input-text').val(param.default).show();
      }
    }
  }

  function updateParamNumbering() {
    tbody.find('.param-number').each(function (idx, elem) {
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
      updateGlobalParamFilter();
    });

    filterList.append(btn);
    updateGlobalParamFilter();
  }

  function clearGlobalParamFilter() {
    filterList.empty();
    filters = [];
    updateGlobalParamFilter();
  }

  function updateGlobalParamFilter() {
    let row = tbody.children('tr');
    let globalParams = dynform.models.rawGlobalParams();

    for (let idx in globalParams) {
      let r = row.eq(parseInt(idx) + 1); // skip default agv_home
      r.show();
      let gp = globalParams[idx];
      for (let id of filters) {
        if (!findInGlobalParam(gp, id)) {
          r.hide();
          break;
        }
      }
    }
  }

  function findInGlobalParam(gp, id) {
    if (id.startsWith('_Station_')) {
      return findStationInGlobalParam(gp, id);
    } else if (id.startsWith('_Register_')) {
      return findRegisterInGlobalParam(gp, id);
    }
  }

  function findStationInGlobalParam(gp, id) {
    let station = id.substr(9);

    if (gp.type !== 'Station') {
      return;
    }

    if (gp.default === station) {
      return true;
    }
  }

  function findRegisterInGlobalParam(gp, id) {
    let register = id.substr(10);

    if (gp.type !== 'Register') {
      return;
    }

    if (gp.default === register) {
      return true;
    }
  }

  dynform.on('models.ready', function () {
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

    var reservedGlobalParams = dynform.models.reservedGlobalParams();
    for (let param of reservedGlobalParams) {
      addReservedParamInlineForm(param);
    }

    var globalParams = dynform.models.rawGlobalParams();
    for (let param of globalParams) {
      addParamInlineForm(param);
    }
    updateParamNumbering();
    addGlobalParam.prop('disabled', false);

    filterUtils.fillOptions(filterField, dynform.models);

    // disable filter if no filter options.
    if (filterField.options.length === 0) {
      filter.hide();
      return;
    }

    let f = dynform.models.rawFilter();
    if (f) {
      filterField.value = f;
      addFilter(f);
    }
  });

  dynform.on('models.dirty', function () {
    updateGlobalParamFilter();
  });

  return dynform;
}
