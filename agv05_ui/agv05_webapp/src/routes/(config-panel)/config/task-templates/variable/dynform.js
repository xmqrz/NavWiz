/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
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
        <th colspan="6">Variables</th>
      </tr>
      <tr>
        <th>#</th>
        <th>Name</th>
        <th>Type</th>
        <th>Description</th>
        <th>Default</th>
        <th>Operation</th>
      </tr>
    </thead>
    <tbody>
    </tbody>
    <tfoot>
      <tr>
        <td colspan="6">
          <button type="button" class="variant-filled btn rounded py-1 form-add-variable">
            <i class="fa-solid fa-plus mr-2"></i> Add variable
          </button>
        </td>
      </tr>
    </tfoot>
  </table>
</div>
`;

var variableInlineFormHtml = `
<tr class="variable-inline">
  <td class="variable-number"></td>
  <td><input type="text" class="input rounded variable-input-name" required/></td>
  <td><select class="select rounded variable-input-type" required>
    <option value="bool">Bool</option>
    <option value="int" selected>Int</option>
    <option value="double">Double</option>
    <option value="str">Str</option>
    <option value="Station">Station</option>
  </select></td>
  <td><input type="text" class="input rounded variable-input-description"/></td>
  <td class="text-center">
    <input type="checkbox" class="checkbox variable-input variable-input-checkbox w-10 h-10"/>
    <input type="text" class="input rounded variable-input variable-input-text"/>
    <input type="number" class="input rounded variable-input variable-input-number" required/>
    <select class="select rounded variable-input variable-input-station" required></select>
  </td>
  <td>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 variable-move-up" tip-title="Move Up">
      <i class="fa fa-arrow-up"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 variable-move-down" tip-title="Move Down">
      <i class="fa fa-arrow-down"></i>
    </button>
    <button type="button" class="variant-filled btn w-7 rounded p-1 my-2 variable-remove" tip-title="Delete">
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

  var addVariable = dynform
    .find('.form-add-variable')
    .on('click', function () {
      var variable =
        dynform.models.rawVariables()[
          dynform.models.addVariable({
            type: 'int'
          })
        ];
      addVariableInlineForm(variable);
      updateVariableNumbering();
      clearVariableFilter();
    })
    .prop('disabled', true);

  function addVariableInlineForm(variable) {
    var inlineForm = $(variableInlineFormHtml);

    var inputName = inlineForm
      .find('.variable-input-name')
      .val(variable.name)
      .on('change', function () {
        variable.name = inputName.val();
        dynform.models.updateVariable(variable);
      });
    var inputType = inlineForm
      .find('.variable-input-type')
      .val(variable.type)
      .on('change', function () {
        variable.type = inputType.val();
        variable.default = variable.type === 'bool' ? false : undefined;
        variable.default = variable.type === 'str' ? '' : variable.default;
        setupVariableInput();
        dynform.models.updateVariable(variable);
      });
    var inputDesc = inlineForm
      .find('.variable-input-description')
      .val(variable.description)
      .on('change', function () {
        variable.description = inputDesc.val();
        dynform.models.updateVariable(variable);
      });

    var inputs = inlineForm.find('.variable-input');
    var inputCheckbox = inlineForm
      .find('.variable-input-checkbox')
      .on('change', updateVariableBool);
    var inputNumber = inlineForm
      .find('.variable-input-number')
      .on('change', updateVariableVal);
    var inputText = inlineForm.find('.variable-input-text').on('change', updateVariableVal);
    var inputStation = inlineForm
      .find('.variable-input-station')
      .on('focus', () => showDropdown(inputStation, prefabOptions.st, variable.default))
      .on('blur', () => hideDropdown(inputStation))
      .on('change', updateVariableVal);
    setupVariableInput();

    inlineForm.find('.variable-move-up').on('click', function () {
      if (dynform.models.moveUpVariable(variable)) {
        inlineForm.insertBefore(inlineForm.prev());
        updateVariableNumbering();
      }
    });
    inlineForm.find('.variable-move-down').on('click', function () {
      if (dynform.models.moveDownVariable(variable)) {
        inlineForm.insertAfter(inlineForm.next());
        updateVariableNumbering();
      }
    });
    inlineForm.find('.variable-remove').on('click', function () {
      if (dynform.models.removeVariable(variable)) {
        inlineForm.remove();
        updateVariableNumbering();
      }
    });

    tbody.append(inlineForm);

    function setupVariableInput() {
      inputs.hide().prop('disabled', true);

      var val = variable.default;
      switch (variable.type) {
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
      }
    }

    function updateVariableBool() {
      /* jshint validthis: true */
      variable.default = $(this).is(':checked');
      dynform.models.updateVariable(variable);
    }

    function updateVariableVal() {
      /* jshint validthis: true */
      variable.default = $(this).val();
      dynform.models.updateVariable(variable);
    }
  }

  function showDropdown(input, options, val) {
    /* populate dropdown options */
    if (input[0].childElementCount <= 1) {
      input.html(options).val(val);
    }
  }

  function hideDropdown(input, empty = '') {
    /* remove dropdown options */
    var v = input.val();
    if (v) {
      input.html(optionHtml(v)).val(v);
    } else {
      input.html(empty).val('');
    }
  }

  function updateVariableNumbering() {
    tbody.find('.variable-number').each(function (idx, elem) {
      $(elem).text(idx + 1);
    });
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
      updateVariableFilter();
    });

    filterList.append(btn);
    updateVariableFilter();
  }

  function clearVariableFilter() {
    filterList.empty();
    filters = [];
    updateVariableFilter();
  }

  function updateVariableFilter() {
    let row = tbody.children('tr');
    let variables = dynform.models.rawVariables();

    for (let idx in variables) {
      let r = row.eq(idx);
      r.show();
      let variable = variables[idx];
      for (let id of filters) {
        if (!findInVariable(variable, id)) {
          r.hide();
          break;
        }
      }
    }
  }

  function findInVariable(variable, id) {
    if (id.startsWith('_Station_')) {
      return findStationInVariable(variable, id);
    }
  }

  function findStationInVariable(variable, id) {
    let station = id.substr(9);

    if (variable.type === 'Station' && variable.default === station) {
      return true;
    }
  }

  dynform.on('models.ready', function () {
    var stationList = dynform.models.stationList();
    prefabOptions.st = '';
    for (let s of stationList) {
      prefabOptions.st += optionHtml(s);
    }

    var variables = dynform.models.rawVariables();
    for (let v of variables) {
      addVariableInlineForm(v);
    }
    updateVariableNumbering();
    addVariable.prop('disabled', false);

    filterUtils.fillOptions(filterField, dynform.models);
    let f = dynform.models.rawFilter();
    if (f) {
      filterField.value = f;
      addFilter(f);
    }
  });

  dynform.on('models.dirty', function () {
    updateVariableFilter();
  });

  return dynform;
}
