/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import taskTemplates from '$lib/shared/services/config/task-templates';
import Select from 'components/Select.svelte';
import taskTemplateFormF from './task-template-form';

var divHtml = `
<div id="div_id_task_triggers" class="grid grid-cols-12">
  <label class="font-medium col-span-2">
    Task Triggers
  </label>
  <div id="task-triggers-section" class="col-span-9 table-container">
  </div>
</div>
`;

var tableHtml = `
<table class="table table-hover">
  <thead>
    <tr>
      <th>Type</th>
      <th>Configuration</th>
      <th>Task Template</th>
    </tr>
  </thead>
  <tbody>
  </tbody>
</table>
`;

var commonConfigurationHtml = `
<div class="space-y-2">
  <div>
    <label>
      <input type="checkbox" class="checkbox task-trigger-abortable"/>
        Auto-abort when new task is available.
    </label>
  </div>
  <div>
    <label class="font-bold">
      Do not trigger when:
    </label>
    <label>
        <input type="checkbox" class="checkbox task-trigger-agv-charging"/>
        AGV is charging.
    </label>
  </div>
  <div>
    <label>
        <input type="checkbox" class="checkbox task-trigger-agv-at-home"/>
        AGV is at home.
    </label>
  </div>
  <div>
      <p>AGV is at station:</p>
      <div class="task-trigger-stations-target"></div>
  </div>
  <div>
    <label>
        <input type="checkbox" class="checkbox task-trigger-low-battery"/>
        AGV&rsquo;s battery is below the minimum level to execute a new task.
    </label>
  </div>
  <div>
    <label>
        <input type="checkbox" class="checkbox task-trigger-active"/>
        The trigger has run once and the trigger condition remains active.
    </label>
  </div>
</div>
</div>
`;

var agvIdleRowHtml = `
<tr>
  <td>AGV Idle</td>
  <td class="task-trigger-configuration space-y-2">
    <div>
      <div>
        <label class="font-bold">
          Timeout
        </label>
        <div class="controls col-xs-12">
          <input type="number" min="3" max="9999" class="input rounded task-trigger-timeout" required/>
          <p class="text-slate-400 italic">Trigger task when AGV is idle for more than this duration (in seconds).</p>
        </div>
      </div>
  </td>
  <td>
    <div class="flex items-center">
      <select class="select flex-1 rounded task-trigger-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="task-trigger-action-parameter"></div>
  </td>
</tr>
`;

var batteryLowRowHtml = `
<tr>
  <td>Battery Low</td>
  <td class="task-trigger-configuration space-y-2">
    <div>
      <div>
        <label class="font-bold">
          Threshold
        </label>
        <div class="controls col-xs-12">
          <input type="number" min="0" max="100" class="input rounded task-trigger-threshold" required/>
          <p class="text-slate-400 italic">Trigger task when AGV&rsquo;s battery below threshold (in percent).</p>
        </div>
      </div>
  </td>
  <td>
    <div class="flex items-center">
      <select class="select flex-1 rounded task-trigger-action"></select>
      <a class="ml-2" target="_blank" title="Open task template">
        <i class="fa-solid fa-up-right-from-square"></i>
      </a>
    </div>
    <div class="task-trigger-action-parameter"></div>
  </td>
</tr>
`;

export default function (form) {
  var div = $(divHtml);
  form.append(div);
  var table = $(tableHtml).appendTo(div.find('#task-triggers-section'));
  table.body = table.find('tbody');

  var agvIdleRow = $(agvIdleRowHtml).appendTo(table.body);
  agvIdleRow.find('.task-trigger-configuration').append(commonConfigurationHtml);

  var batteryLowRow = $(batteryLowRowHtml).appendTo(table.body);
  batteryLowRow.find('.task-trigger-configuration').append(commonConfigurationHtml);

  div.on('models.loaded', function () {
    var agvIdle = div.models.agvIdle();
    var batteryLow = div.models.batteryLow();
    var taskTemplateMetas = div.models.taskTemplateMetas();
    var stationList = div.models.stationList();
    var registerList = div.models.registerList();

    var taskTemplateForm = taskTemplateFormF(taskTemplateMetas, stationList, registerList);
    var prefabOptions = taskTemplateForm.prefabOptions();

    // load agv idle row
    var agvIdleTimeout = agvIdleRow
      .find('.task-trigger-timeout')
      .val(agvIdle.timeout)
      .on('change', function () {
        agvIdle.timeout = agvIdleTimeout.val();
      });
    setupCommonConfiguration(agvIdleRow, agvIdle);

    // load battery low row
    var batteryLowThreshold = batteryLowRow
      .find('.task-trigger-threshold')
      .val(batteryLow.threshold)
      .on('change', function () {
        batteryLow.threshold = batteryLowThreshold.val();
      });
    setupCommonConfiguration(batteryLowRow, batteryLow);

    function setupCommonConfiguration(row, data) {
      let abortable = row
        .find('.task-trigger-abortable')
        .prop('checked', !!data.abortable)
        .on('change', function () {
          data.abortable = abortable.is(':checked');
        });
      let ignoreCharging = row
        .find('.task-trigger-agv-charging')
        .prop('checked', !!data.ignore_charging)
        .on('change', function () {
          data.ignore_charging = ignoreCharging.is(':checked');
        });
      let ignoreHome = row
        .find('.task-trigger-agv-at-home')
        .prop('checked', !!data.ignore_home)
        .on('change', function () {
          data.ignore_home = ignoreHome.is(':checked');
        });

      var inputTaskTriggerStationsTarger = row.find('.task-trigger-stations-target');
      let stationListOptions = [];
      stationList.forEach(function (value, i) {
        stationListOptions.push([i, value]);
      });
      var ignoreStations = new Select({
        target: inputTaskTriggerStationsTarger[0],
        props: {
          buttonClass: 'rounded py-1 text-sm variant-form h-7',
          class: 'w-[250px]',
          enableFilter: false,
          startEmpty: true,
          multiple: true,
          value: data.ignore_stations,
          options: stationListOptions
        }
      });
      ignoreStations.$on('change', function () {
        data.ignore_stations = ignoreStations.value || [];
      });

      let ignoreLowBatt = row
        .find('.task-trigger-low-battery')
        .prop('checked', !!data.ignore_low_battery)
        .on('change', function () {
          data.ignore_low_battery = ignoreLowBatt.is(':checked');
        });
      let ignoreTriggerActive = row
        .find('.task-trigger-active')
        .prop('checked', !!data.ignore_trigger_active)
        .on('change', function () {
          data.ignore_trigger_active = ignoreTriggerActive.is(':checked');
        });
      let action = row
        .find('.task-trigger-action')
        .html(prefabOptions.ac)
        .val(data.action.skillId)
        .on('change', function () {
          var _skillId = data.action.skillId;
          data.action.skillId = action.val();
          setupLink(action.next(), data.action);
          taskTemplateForm.setupParamInputs(action.parent().next(), data.action, {
            _skillId: _skillId || ''
          });
        });
      if (data.action.skillId && !taskTemplateMetas[data.action.skillId]) {
        action.prepend(`<option value="${data.action.skillId}">(invalid)</option>`);
      }
      setupLink(action.next(), data.action);
      taskTemplateForm.setupParamInputs(action.parent().next(), data.action);
    }

    function setupLink(link, action) {
      if (action.skillId && action.skillId.indexOf('_ttpk_') === 0) {
        link.attr('href', taskTemplates.editUrl(action.skillId.slice(6))).show();
      } else {
        link.removeAttr('href').hide();
      }
    }
  });

  return div;
}
