/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

var paramHeaderHtml = `
<div class="row form-group-sm">
  <label class="col-sm-12 control-label font-bold text-left">Parameters:</label>
</div>
`;

var paramRowHtml = `
<div class="grid grid-cols-4 space-y-1">
  <span class="col-span-2 control-label my-auto text-left"></span>
  <div class="col-span-2 flex justify-center">
    <input class="input form-control rounded" tip-placement="top"/>
    <input type="checkbox" class="checkbox form-control p-5" tip-placement="top"/>
    <select class="select form-control rounded" tip-placement="top"/>
  </div>
</div>
`;

export default function (taskTemplateMetas, stationList, registerList) {
  var prefabOptions = {};
  prefabOptions.ac = $('<select>');
  prefabOptions.ac.append('<option value="">---------</option>');
  for (let k in taskTemplateMetas) {
    let d = taskTemplateMetas[k];
    let option = $('<option>');
    option.val(d.id).text(d.name);
    prefabOptions.ac.append(option);
  }
  prefabOptions.ac = prefabOptions.ac.html();

  prefabOptions.st = '';
  for (let s of stationList) {
    prefabOptions.st += `<option value="${s}">${s}</option>`;
  }

  prefabOptions.reg = '';
  for (let r of registerList) {
    prefabOptions.reg += `<option value="${r}">${r}</option>`;
  }

  function setupParamInputs(outer, action, reset) {
    outer.empty();
    if (!action.skillId) {
      return;
    }

    var skillDesc;
    var formatLabel;
    if (action.skillId.indexOf('_ttpk_') === 0) {
      skillDesc = taskTemplateMetas[action.skillId];
      formatLabel = _.identity;
    }
    if (!skillDesc || !skillDesc.params) {
      return;
    }

    if (reset && action.skillId !== reset._skillId) {
      var _skillDesc;
      if (reset._skillId.indexOf('_ttpk_') === 0) {
        _skillDesc = taskTemplateMetas[reset._skillId];
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
      paramRow.find('.control-label').html(formatLabel(paramDesc.name));
      var paramInputs = paramRow
        .find('.form-control')
        .attr('tip-title', paramDesc.description);

      _updateParamInput();
      outer.append(paramRow);

      function _updateParamInput() {
        var paramVal = action.params[paramDesc.key];
        paramInputs.prop('required', false).hide().off('change');

        if (paramDesc.type === 'bool') {
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
          paramInputs
            .last()
            .html(prefabOptions.st)
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        } else if (paramDesc.type === 'Register') {
          paramInputs
            .last()
            .html(prefabOptions.reg)
            .prop('required', true)
            .val(paramVal)
            .on('change', _updateParamVal)
            .show();
        }
      }

      function _updateParamVal() {
        /* jshint validthis: true */
        action.params[paramDesc.key] = $(this).val();
      }

      function _updateParamBool() {
        /* jshint validthis: true */
        action.params[paramDesc.key] = $(this).is(':checked');
      }
    });

    if (!skillDesc.params.length) {
      outer.text('-');
    }
  }
  return {
    setupParamInputs: setupParamInputs,
    prefabOptions: () => prefabOptions
  };
}
