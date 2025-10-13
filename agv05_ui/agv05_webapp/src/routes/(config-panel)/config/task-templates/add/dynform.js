/* Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import { InputChip } from '@skeletonlabs/skeleton';

import taskTemplates from '$lib/shared/services/config/task-templates';

var dynformHtml = `
<div>
  <div class="table-container p-3">
    <span class="font-medium">Params</span>
    <table class="table table-hover">
      <thead>
        <tr>
          <th>Name</th>
          <th>Type</th>
          <th>Description</th>
          <th>Default</th>
          <th>Min</th>
          <th>Max</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody></tbody>
      <tfoot>
        <tr>
          <td colspan="7">
            <button type="button" class="variant-filled btn rounded py-1 param-add">
              <i class="fa-solid fa-plus mr-2"></i>
              Add param
            </button>
          </td>
        </tr>
      </tfoot>
    </table>
  </div>
  <div class="p-3 grid lg:grid-cols-2">
    <label class="label">
      <span class="font-medium">Outcomes</span>
      <div class="task-template-outcome"></div>
    </label>
  </div>
  <div class="px-3 pt-3 font-medium">Structure</div>
  <div class="px-3 view-preserve hidden">
    <strong class="warn-corrupted hidden text-error-600-300-token">Some structure might have corrupted.</strong>
    View <a class="anchor no-underline task-template-last-saved" target="_blank">last saved</a>
    <span class="cached-text hidden">
    or <a class="anchor no-underline task-template-last-cached" target="_blank">last cached</a>
    </span>
    original structure.
  </div>
</div>
`;

var paramInlineFormHtml = `
<tr>
  <td><input type="text" class="input rounded param-input-name" required/></td>
  <td><select class="select rounded param-input-type" required>
    <option value="bool">Bool</option>
    <option value="int" selected>Int</option>
    <option value="double">Double</option>
    <option value="str">Str</option>
    <option value="Station">Station</option>
    <option value="Register">Register</option>
    <option value="vbool">Bool Variable</option>
    <option value="vint">Int Variable</option>
    <option value="vdouble">Double Variable</option>
    <option value="vstr">Str Variable</option>
    <option value="vStation">Station Variable</option>
  </select></td>
  <td><input type="text" class="input rounded param-input-description"/></td>
  <td><input type="number" class="input rounded param-input-default"/></td>
  <td><input type="number" class="input rounded param-input-min"/></td>
  <td><input type="number" class="input rounded param-input-max"/></td>
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

export default function (viz) {
  var $viz = $(viz.node());
  var dynform = $(dynformHtml);
  var tbody = dynform.find('tbody');

  /* Section: Param */
  dynform.find('.param-add').on('click', function (e) {
    var param =
      viz.models.rawParams()[
        viz.models.addParam({
          type: 'int'
        })
      ];
    addParamInlineForm(param);
    viz.scene.activeObject.set(null, null);
    e.preventDefault();
  });

  function addParamInlineForm(param) {
    var inlineForm = $(paramInlineFormHtml);

    var inputName = inlineForm
      .find('.param-input-name')
      .val(param.name)
      .on('change', function () {
        viz.models.updateParam(param, {
          name: inputName.val()
        });
        viz.scene.activeObject.set(null, null);
      });
    var inputType = inlineForm
      .find('.param-input-type')
      .val(param.type)
      .on('change', function () {
        var update = {
          type: inputType.val()
        };
        if (['int', 'double'].indexOf(update.type) < 0) {
          update.default = undefined;
          update.min = undefined;
          update.max = undefined;
        }
        updateInputsDefMinMax(update.type);
        viz.models.updateParam(param, update);
        viz.scene.activeObject.set(null, null);
      });
    var inputDescription = inlineForm
      .find('.param-input-description')
      .val(param.description)
      .on('change', function () {
        viz.models.updateParam(param, {
          description: inputDescription.val()
        });
      });
    var inputDefault = inlineForm
      .find('.param-input-default')
      .val(param.default)
      .on('change', function () {
        viz.models.updateParam(param, {
          default: Number.parseFloat(inputDefault.val())
        });
      });
    var inputMin = inlineForm
      .find('.param-input-min')
      .val(param.min)
      .on('change', function () {
        viz.models.updateParam(param, {
          min: Number.parseFloat(inputMin.val())
        });
      });
    var inputMax = inlineForm
      .find('.param-input-max')
      .val(param.max)
      .on('change', function () {
        viz.models.updateParam(param, {
          max: Number.parseFloat(inputMax.val())
        });
      });
    updateInputsDefMinMax(param.type);

    function updateInputsDefMinMax(newType) {
      if (['int', 'double'].indexOf(newType) >= 0) {
        var step = newType === 'int' ? 1 : 'any';
        inputDefault.prop('disabled', false).attr('step', step);
        inputMin.prop('disabled', false).attr('step', step);
        inputMax.prop('disabled', false).attr('step', step);
      } else {
        inputDefault.prop('disabled', true).val(undefined);
        inputMin.prop('disabled', true).val(undefined);
        inputMax.prop('disabled', true).val(undefined);
      }
    }

    inlineForm.find('.param-move-up').on('click', function (e) {
      viz.models.moveUpParam(param);
      inlineForm.insertBefore(inlineForm.prev());
      e.preventDefault();
    });
    inlineForm.find('.param-move-down').on('click', function (e) {
      viz.models.moveDownParam(param);
      inlineForm.insertAfter(inlineForm.next());
      e.preventDefault();
    });
    inlineForm.find('.param-remove').on('click', function (e) {
      viz.models.removeParam(param);
      inlineForm.remove();
      viz.scene.activeObject.set(null, null);
      e.preventDefault();
    });

    tbody.append(inlineForm);
  }

  $viz.on('models.ready', function () {
    var id = viz.models.id();
    var params = viz.models.rawParams();
    for (let param of params) {
      addParamInlineForm(param);
    }

    if (!viz.options.editable) {
      dynform.find('input,select').prop('disabled', true);
      dynform.find('button').prop('disabled', true).off('click');
    }
    if (id && !viz.options.preserve) {
      var viewPreserve = dynform.find('.view-preserve');
      if (viz.models.corrupted()) {
        viewPreserve.find('.warn-corrupted').show();
      }
      if (viz.models.isCached()) {
        viewPreserve.find('.cached-text').show();
      }
      viewPreserve
        .find('.task-template-last-saved')
        .attr('href', taskTemplates.viewSavedUrl(id));
      viewPreserve
        .find('.task-template-last-cached')
        .attr('href', taskTemplates.viewCachedUrl(id));
      viewPreserve.show();
    }
  });

  /* Section: Outcomes */
  var outcomeTarget = dynform.find('.task-template-outcome');
  var outcomeValue = ['End'];
  var outcome = new InputChip({
    target: outcomeTarget[0],
    props: {
      name: 'task_template_outcome',
      placeholder: ''
    }
  });
  outcome.$set({ value: outcomeValue });
  outcome.$on('add', outcomeUpdated);
  outcome.$on('remove', outcomeUpdated);

  $viz.on('models.ready models.updated', function () {
    let topLvl = viz.models.topLvl();
    outcomeValue = [...viz.models.rawOutcomes()];

    outcome.$set({
      value: outcomeValue,
      disabled: topLvl
    });
  });

  function outcomeUpdated() {
    viz.models.updateOutcomes(outcomeValue);
  }

  return dynform;
}
