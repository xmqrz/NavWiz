/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import { popup } from '@skeletonlabs/skeleton';

import laserInspectorF from './laser-inspector';

var menuHtml = `
<div class="space-y-3">
  <div class="flex space-x-3 items-center">
    <label class="label font-semibold" for="active-lidar">Lidar:</label>
    <select id="active-lidar" class="select px-3 text-xs rounded-token">
    </select>
  </div>
  <div class="flex space-x-3 items-center">
    <label class="label font-semibold" for="min-activation">Min Activation:</label>
    <input id="min-activation" class="input text-xs flex-1" type="number" step="1" min="1" max="100"></input>
  </div>
  <div class="flex space-x-3 items-center">
    <label class="label font-semibold" for="active-area">Area:</label>
    <select id="active-area" class="select px-3 text-xs rounded-token">
    </select>
  </div>
  <div class="flex space-x-3 items-center">
    <label class="label font-semibold" for="active-region">Region:</label>
    <select class="select px-3 text-xs rounded-token" id="active-region">
      <option value="near" selected>Near</option>
      <option value="middle">Middle</option>
      <option value="far">Far</option>
    </select>
  </div>
</div>
`;

var tableHtml = `
<div class="table-container coord-list">
  <table class="table table-hover">
    <thead>
      <tr class="text-sm">
        <th>X (mm)</th>
        <th>Y (mm)</th>
        <th width="70px">
          <div class="btn-group variant-surface-filled rounded">
            <button type="button" id="reset-region" tip-title="Reset to defaults">
              <i class="fa fa-eraser"></i>
            </button>
            <button type="button" class="utils-btn">
              <i class="fa fa-ellipsis-h"></i>
            </button>
            <ul class="z-50 bg-surface-700-200-token rounded" data-popup="utils-popup">
              <li><div role="toolbar" aria-label="...">
                <div class="btn-group variant-filled rounded">
                  <button type="button" id="copy-region" tip-title="Copy">
                    <i class="fa-solid fa-copy"></i>
                  </button>
                  <button type="button" id="paste-region" tip-title="Paste">
                    <i class="fa-solid fa-clipboard"></i>
                  </button>
                </div>
              </div></li>
            </ul>
          </div>
        </th>
      </tr>
    </thead>
    <tbody>
    </tbody>
  </table>
</div>
`;

var tableRowHtml = `
<tr>
  <td><input class="input coord-x" type="number" step="1"></input></td>
  <td><input class="input coord-y" type="number" step="1"></input></td>
  <td>
    <button type="button" class="btn variant-filled rounded text-xs coord-add"><i class="fa fa-chevron-down"></i></button>
    <button type="button" class="btn variant-filled rounded text-xs coord-remove"><i class="fa fa-trash"></i></button>
  </td>
</tr>
`;

var submitButtonHtml = `
<button type="submit" class="btn variant-filled-primary">Submit</button>
`;

export default function (viz, container) {
  var $viz = $(viz.node());

  // region selection menu
  var menu = $(menuHtml);
  var optMinActivation = menu.find('#min-activation');
  optMinActivation.on('change', function () {
    viz.models.setMinActivation(parseInt(optMinActivation.val()));
    regionUpdated();
  });
  var optLidar = menu.find('#active-lidar');
  optLidar.on('change', function () {
    viz.models.setActiveLidar(parseInt(optLidar.val()));
    viz.scene.setActiveCoord(-1);
    optMinActivation.val(viz.models.getMinActivation());
    regionUpdated();
  });
  var optArea = menu.find('#active-area');
  optArea.on('change', function () {
    viz.models.setActiveArea(parseInt(optArea.val()));
    viz.scene.setActiveCoord(-1);
    regionUpdated();
  });
  var optRegion = menu.find('#active-region');
  optRegion.on('change', function () {
    viz.scene.setActiveRegion(optRegion.val());
    viz.scene.setActiveCoord(-1);
    regionUpdated();
  });

  container.append(menu);

  // live detection
  var laserInspector = laserInspectorF(viz, container);

  // coord list table
  var table = $(tableHtml);
  container.append(table);
  var tbody = table.find('tbody');
  var utils = table.find('.utils-btn');
  popup(utils[0], {
    event: 'click',
    target: 'utils-popup',
    placement: 'bottom'
  });
  table.find('#reset-region').on('click', resetRegion);
  table.find('#copy-region').on('click', copyRegion);
  var pasteRegionBtn = table.find('#paste-region').on('click', pasteRegion);
  var activeRow = -1;

  $viz.on('models.loaded', function () {
    // update laser option
    let lidarOptions = viz.models.rawLidarOptions();
    let activeOption = viz.models.rawActiveOption();
    let lidarData = viz.models.getLidarData(activeOption[0]);
    let minActivation = viz.models.getMinActivation();

    optLidar.empty();
    optArea.empty();
    for (let o of lidarOptions) {
      let option = $('<option>');
      option.val(o[0]).text(o[1]);
      optLidar.append(option);
    }
    for (let i in lidarData.areas) {
      let a = lidarData.areas[i];
      let option = $('<option>');
      option.val(i).text(a.name);
      optArea.append(option);
    }
    optMinActivation.val(minActivation);
    optLidar.val(activeOption[0]);
    optArea.val(activeOption[1]);
    regionUpdated();
  });

  $viz.on('scene.activeRegion', function (evt, activeRegion) {
    optRegion.val(activeRegion);
    activeRow = -1;
    regionUpdated();
  });
  $viz.on('scene.activeCoord', function (evt, activeCoord) {
    activeRow = activeCoord;
  });
  $viz.on('scene.updateTable', updateTable);

  function updateTable() {
    var data = viz.models.rawRegions()[optRegion.val()] || [];
    var tr = tbody.children('tr:not(.add-row)');

    // create new rows (enter)
    for (let i = tr.length; i < data.length; ++i) {
      let tr0 = $(tableRowHtml);
      tr0
        .find('.coord-x')
        .data('i', i)
        .data('j', 0)
        .on('click', activateCoord)
        .on('change', updateCoord);
      tr0
        .find('.coord-y')
        .data('i', i)
        .data('j', 1)
        .on('click', activateCoord)
        .on('change', updateCoord);
      tr0.find('.coord-remove').data('i', i).on('click', removeCoord);
      tr0.find('.coord-add').data('i', i).on('mousedown', addCoord);
      tbody.append(tr0);
    }
    tr = tbody.children('tr:not(.add-row)');

    // update row values (update)
    for (let i = 0; i < data.length; ++i) {
      let tr0 = $(tr[i]);
      if (i < data.length) {
        tr0.find('.coord-x').val(data[i][0]);
        tr0.find('.coord-y').val(data[i][1]);
        if (data.length > 3) {
          tr0.find('.coord-remove').show();
        } else {
          tr0.find('.coord-remove').hide();
        }
      }
      if (i === activeRow && i < data.length) {
        tr0.addClass('active');
        tr0[0].scrollIntoView({
          block: 'nearest',
          inline: 'nearest',
          behavior: 'smooth'
        });
      } else {
        tr0.removeClass('active');
      }
      tr0.removeClass('has-error'); // clear error once validated or discarded
    }

    // remove extra rows (exit)
    tr.slice(data.length).remove();
    regionUpdated();
  }

  function activateCoord() {
    /* jshint validthis: true */
    viz.scene.setActiveCoord($(this).data('i'));
  }

  function updateCoord() {
    /* jshint validthis: true */
    var $this = $(this);
    // var len = viz.models.rawRegions()[optRegion.val()].length;

    if (!$this.hasClass('add-row')) {
      $this.parent().parent().addClass('has-error'); // mark change as error first
      let v = Number.parseInt($this.val());
      if (Number.isInteger(v)) {
        viz.models.updateCoord(optRegion.val(), $this.data('i'), $this.data('j'), v);
      }
    } else {
      let x = $this.parent().parent().find('.coord-x').val();
      let y = $this.parent().parent().find('.coord-y').val();
      if (x && y) {
        $this.parent().parent().addClass('has-error'); // mark change as error first
      }
      x = Number.parseInt(x);
      y = Number.parseInt(y);
      if (Number.isInteger(x) && Number.isInteger(y)) {
        let result = viz.models.addCoord(optRegion.val(), [x, y], $this.data('i'));
        if (result !== undefined) {
          // Add row next line
          let nextInput = $this.parent().parent().next().find('.coord-x');
          addCoord.call(nextInput[0]);
          activateCoord.call(nextInput[0]);
          $this.parent().parent().remove();
        }
      }
    }
    regionUpdated();
  }

  function addCoord(event) {
    /* jshint validthis: true */
    if (event) {
      // If from mousedown event.
      event.preventDefault();
    }
    let $this = $(this);
    let tr = $(tableRowHtml);
    tbody.children('tr.add-row').remove();
    $this.parent().parent().after(tr);

    let i = $this.data('i') + 1;
    tr.find('.coord-remove').hide();
    tr.find('.coord-add').hide();
    tr.find('.coord-x')
      .addClass('add-row')
      .data('i', i)
      .data('j', 0)
      .on('change', updateCoord)
      .val(null)
      .trigger('focus');
    tr.find('.coord-y')
      .addClass('add-row')
      .data('i', i)
      .data('j', 1)
      .on('change', updateCoord)
      .val(null);
    tr.on('focusout', function (event) {
      if (this.contains(event.relatedTarget)) {
        return;
      }
      tr.remove();
    });
    tr.addClass('add-row');
    activateCoord.call($this.parent().parent().find('.coord-x')[0]);
    regionUpdated();
  }

  function removeCoord() {
    /* jshint validthis: true */
    let result = viz.models.removeCoord(optRegion.val(), $(this).data('i'));
    if (result === false) {
      //delete result to invalid polygon.
      tbody.children('tr:not(.add-row)').removeClass('has-error');
      $(this).parent().parent().addClass('has-error');
    }
    regionUpdated();
  }

  function resetRegion() {
    if (!viz.models.hasDefault()) {
      return window.alert(
        'Cannot obtain default configuration when the robot controller is not running.\nPlease refresh the page after the robot controller is restarted.'
      );
    }

    let region = optRegion.val();
    if (window.confirm('Reset region "' + region + '" to its default configuration?')) {
      viz.models.resetToDefault(region);
    }
    regionUpdated();
  }

  function copyRegion() {
    let region = optRegion.val();
    viz.models.copyRegion(region);
    if (viz.models.hasClipboard()) {
      pasteRegionBtn.removeClass('disabled');
    }
  }

  function pasteRegion() {
    if (!viz.models.hasClipboard()) {
      return;
    }
    let region = optRegion.val();
    viz.models.pasteRegion(region);
    regionUpdated();
  }

  function updateLaserPose(data) {
    laserInspector.updateLaserPose(data);
  }

  function updateLaserScan(data) {
    laserInspector.updateLaserScan(data);
  }

  function regionUpdated() {
    laserInspector.regionUpdated();
  }

  // submit button
  var submitButton = $(submitButtonHtml);
  submitButton.on('click', function () {
    viz.editor.save();
  });
  container.append(submitButton);

  viz.table = {
    updateLaserPose: updateLaserPose,
    updateLaserScan: updateLaserScan,
    getActiveRegion: () => optRegion.val()
  };

  updateTable();
}
