/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import { get } from 'svelte/store';
import { popup } from '@skeletonlabs/skeleton';

import { search } from 'stores/search';
import EditMode from './scene/edit-mode';
import Junction from './models/junction';
import NoRotateZone from './models/no-rotate-zone';
import LocHintZone from './models/loc-hint-zone';
import Path from './models/path';
import Station from './models/station';
import changesetViewF from './changeset-view';

var connectorStraightIconSvg = `<line x1="0" y1="1" x2="10" y2="13"></line>`;
var connectorRightAngledIconSvg = `<path d="M 0 2 H 6 A 3 3,0,0,1,9 5 V 14" fill="none"></path>`;
var connectorSCurvedIconSvg = `<path d="M 1 0 V 5 C 1 7,9 7,9 9 V 14" fill="none"></path>`;
var noRotateZoneIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="yellow" fill-opacity="0.5" stroke="gold"></rect>`;
var locHintZoneIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="blue" fill-opacity="0.5" stroke="blue"></rect>`;

/* Toolbar template */
var toolbarHtml =
  `
<div class="agv05-toolbar">
` +
  /* Group 1: Save & Changesets */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none !variant-filled-primary toolbar-save" disabled tip-title="Save">
    <i class="fa fa-floppy-disk"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-changesets" tip-title="Revision History">
    <i class="fa fa-history"></i>
  </button>
  <button type="button" class="btn rounded-none toolbar-apply-changesets" style="display: none;">
    Apply Changeset
  </button>
</div>
` +
  /* Group 2: Connectors, Station, No-Rotate Zone & Ruler */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-pointer active" tip-title="Select (Esc)">
    <i class="fa fa-mouse-pointer"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station" tip-title="Station (F2)">
    <i class="fa fa-plus-square"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-connector toolbar-connector-straight" tip-title="Straight Path (Alt+1)">
    <i><svg width="10" height="14" class="stroke-[black]">${connectorStraightIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-connector toolbar-connector-right-angled" tip-title="Right-angled Path (Alt+2)">
    <i><svg width="10" height="14" class="stroke-[black]">${connectorRightAngledIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-connector toolbar-connector-s-curved" tip-title="S-curved Path (Alt+3)">
    <i><svg width="10" height="14" class="stroke-[black]">${connectorSCurvedIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-connector-menu">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-connector-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn toolbar-connector-straight toolbar-connector-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black]">${connectorStraightIconSvg}</svg></span>
      <span class="flex-auto">Straight Path (Alt+1)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-connector-right-angled toolbar-connector-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black]">${connectorRightAngledIconSvg}</svg></span>
      <span class="flex-auto">Right-angled Path (Alt+2)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-connector-s-curved toolbar-connector-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black]">${connectorSCurvedIconSvg}</svg></span>
      <span class="flex-auto">S-curved Path (Alt+3)</span>
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-zone toolbar-loc-hint-zone" tip-title="Location Hint Zone (Alt+6)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${locHintZoneIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-no-rotate-zone" tip-title="No-Rotate Zone (Alt+7)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${noRotateZoneIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
</div>
<!--<div class="btn-group" role="group" aria-label="...">
  <a class="btn btn-default toolbar-ruler" data-toggle="tooltip" data-placement="bottom" tip-title="Measure Distance (Alt+0)">
    <span>&#x1F4CF;&#xFE0E;</span>
  </a>
</div>-->
` +
  /* Todo: Group 3: Connected Components */
  /*
  `
  <div class="btn-group rounded">
    <button type="button" class="btn-icon rounded-none toolbar-connected-components" tip-title="Connected Components">
      <i class="fa fa-object-ungroup"></i>
    </button>
  </div>
  ` +
  */
  /* Group 4: Undo, Redo & Clear Everything */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-clear-all" tip-title="Clear Everything">
    <i class="fa fa-eraser"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-utilities">
    <i class="fa fa-ellipsis-h"></i>
  </button>
</div>
<ul class="z-50 bg-gray-400 border border-gray-400 rounded" data-popup="toolbar-utilities-popup">
  <li><div role="toolbar" aria-label="...">
    <div class="btn-group rounded">
      <button type="button" class="btn-icon rounded-none toolbar-undo" disabled="disabled" tip-title="Undo (Ctrl+Z)">
        <i class="fa-solid fa-rotate-left"></i>
      </button>
      <button type="button" class="btn-icon rounded-none toolbar-redo" disabled="disabled" tip-title="Redo (Ctrl+Y)">
        <i class="fa-solid fa-rotate-right"></i>
      </button>
    </div>
  </div></li>
</ul>
` +
  /* Group 5: Zooming */
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
  /* Group 6: Selection properties */
  `
<span class="divider-vertical h-4 toolbar-junction-properties"></span>
<div class="inline-block toolbar-junction-properties">Junction: </div>
<div class="inline-flex toolbar-junction-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">RFID</div>
    <input type="text" id="toolbar-junction-rfid" placeholder="rfid" />
  </div>
</div>
<span class="divider-vertical h-4 toolbar-path-properties"></span>
<div class="inline-block toolbar-path-properties">Path: </div>
<div class="btn-group rounded toolbar-path-properties">
  <button type="button" class="btn-icon rounded-none toolbar-path-flow" tip-title="Change Flow">
    <i class="fa fa-exchange"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-path-facing" tip-title="Change Forward / Reverse Motion">
    <i class="fa fa-shield fa-flip-vertical"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-path-bend" tip-title="Change Bending">
    <i class="fa fa-random"></i>
  </button>
</div>
<div class="inline-flex toolbar-path-properties">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Speed</div>
    <input type="number" class="toolbar-path-speed" step="0.01" min="0.01" max="3.0" tip-title="Speed in m/s"></input>
    <select class="toolbar-select toolbar-path-speed-param" tip-title="Speed in m/s"></select>
    <div class="input-group-shim text-black">
      <label tip-title="Inherit Map Parameter" tip-offset="14">
        <input type="checkbox" class="checkbox toolbar-path-speed-inherit"></input>
        Inherit
      </label>
    </div>
  </div>
</div>
<div class="btn-group rounded toolbar-path-properties">
  <button type="button" class="btn-icon rounded-none toolbar-path-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical h-4 toolbar-station-properties"></span>
<div class="inline-block toolbar-station-properties">Station: </div>
<div class="inline-flex toolbar-station-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Name</div>
    <input type="text" id="toolbar-station-name" placeholder="Name"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-station-properties">
  <button type="button" class="btn-icon rounded-none toolbar-station-direction" tip-title="Change Direction">
    <i class="fa fa-compass"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station-next-overlapped" tip-title="Next Overlapped Station">
    <i class="fa fa-arrow-down"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station-find-all-references" tip-title="Find All References">
    <i class="fa-solid fa-up-right-from-square"></i>
  </button>
</div>
<div class="btn-group rounded toolbar-station-properties">
  <button type="button" class="btn-icon rounded-none toolbar-station-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical h-4 toolbar-no-rotate-zone-properties"></span>
<div class="inline-block toolbar-no-rotate-zone-properties">No-Rotate Zone: </div>
<div class="btn-group rounded toolbar-no-rotate-zone-properties">
  <button type="button" class="btn-icon rounded-none toolbar-no-rotate-zone-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical toolbar-loc-hint-zone-properties"></span>
<div class="inline-block toolbar-loc-hint-zone-properties">Location Hint Zone: </div>
<div class="inline-flex toolbar-loc-hint-zone-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Name</div>
    <input type="text" id="toolbar-loc-hint-zone-name" placeholder="Name"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-loc-hint-zone-properties">
  <button type="button" class="btn-icon rounded-none toolbar-loc-hint-zone-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical h-4 toolbar-multi-properties"></span>
<div class="inline-block toolbar-multi-properties">Multi: </div>
<div class="btn-group rounded toolbar-multi-properties">
  <button type="button" class="btn-icon rounded-none toolbar-multi-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
` +
  /* Group 7: Misc */
  `
<span class="divider-vertical h-4 toolbar-search-tools"></span>
<div class="inline-block toolbar-search-tools">Search: </div>
<div class="inline-flex toolbar-search-tools">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <datalist id="toolbar-search-tools-list"></datalist>
    <input type="text" id="toolbar-search-tools-field" placeholder="Search" list="toolbar-search-tools-list"></input>
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
  toolbar.insertBefore($viz);

  var titleInfo = $viz.parent().siblings('.title-info');
  var changesetView = changesetViewF(viz);

  /* Group 1: Save & Changesets */
  var save = toolbar.find('.toolbar-save').on('click', function () {
    hideFullscreen();
    viz.editor.save();
  });

  // TODO: setup changeset.
  var applyChangeset = toolbar.find('.toolbar-apply-changesets').on('click', function () {
    viz.editor.save();
  });
  var changeset = toolbar.find('.toolbar-changesets').on('click', function () {
    if (changeset.hasClass('active')) {
      hideChangesetView();
      viz.editor.load();
      viz.scene.activeObject.set(null);
    } else {
      showChangesetView();
    }
  });

  function hideChangesetView() {
    applyChangeset.hide();
    changesetView.hide();
    changeset.removeClass('active');
    enable();
  }

  function showChangesetView() {
    applyChangeset.show();
    changesetView.show();
    changeset.addClass('active');
    disable();
  }

  $viz.on('models.dirty', function () {
    save.removeAttr('disabled');
    changeset.prop('disabled', true);
    if (changeset.hasClass('active')) {
      hideChangesetView();
    }
  });

  /* Group 2: Connectors, Station, No-Rotate Zone & Ruler */
  var pointer = toolbar.find('.toolbar-pointer').on('click', activatePointer);
  var station = toolbar.find('.toolbar-station').on('click', activateStation);
  var connectors = toolbar.find('.toolbar-connector');
  var connectorMenu = toolbar.find('.toolbar-connector-menu');
  var connectorOptions = toolbar.find('.toolbar-connector-option');
  popup(connectorMenu[0], {
    event: 'click',
    target: 'toolbar-connector-menu-popup',
    placement: 'bottom'
  });
  var connectorStraight = toolbar
    .find('.toolbar-connector-straight')
    .on('click', activateConnectorStraight);
  var connectorRightAngled = toolbar
    .find('.toolbar-connector-right-angled')
    .on('click', activateConnectorRightAngled);
  var connectorSCurved = toolbar
    .find('.toolbar-connector-s-curved')
    .on('click', activateConnectorSCurved);
  var noRotateZone = toolbar.find('.toolbar-no-rotate-zone').on('click', activateNoRotateZone);
  var locHintZone = toolbar.find('.toolbar-loc-hint-zone').on('click', activateLocHintZone);
  var ruler = toolbar.find('.toolbar-ruler').on('click', activateRuler);

  function activatePointer() {
    pointer.addClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.POINTER);
    viz.zoom.enable();
  }

  function activateStation() {
    station.addClass('active');
    pointer.removeClass('active');
    connectors.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.STATION);
    viz.zoom.disablePan();
  }

  function activateConnectorStraight() {
    connectorOptions.removeClass('active');
    connectors.hide();
    connectorStraight.css('display', 'inline-flex');
    connectorStraight.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.CONNECTOR_STRAIGHT);
    viz.zoom.disablePan();
  }

  function activateConnectorRightAngled() {
    connectorOptions.removeClass('active');
    connectors.hide();
    connectorRightAngled.css('display', 'inline-flex');
    connectorRightAngled.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.CONNECTOR_RIGHT_ANGLED);
    viz.zoom.disablePan();
  }

  function activateConnectorSCurved() {
    connectorOptions.removeClass('active');
    connectors.hide();
    connectorSCurved.css('display', 'inline-flex');
    connectorSCurved.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.CONNECTOR_S_CURVED);
    viz.zoom.disablePan();
  }

  function activateNoRotateZone() {
    noRotateZone.addClass('active');
    locHintZone.removeClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.NO_ROTATE_ZONE);
    viz.zoom.disablePan();
  }

  function activateLocHintZone() {
    locHintZone.addClass('active');
    noRotateZone.removeClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    ruler.removeClass('active');
    viz.scene.setEditMode(EditMode.LOC_HINT_ZONE);
    viz.zoom.disablePan();
  }

  function activateRuler() {
    ruler.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    noRotateZone.removeClass('active');
    locHintZone.removeClass('active');
    locHintZone.removeClass('active');
    viz.scene.setEditMode(EditMode.RULER);
    viz.zoom.disablePan();
  }

  $viz.on('scene.editMode', function () {
    switch (viz.scene.mode) {
      case EditMode.POINTER:
        activatePointer();
        break;
      case EditMode.NO_ROTATE_ZONE:
        activateNoRotateZone();
        break;
    }
  });

  function onKeyupAO(event) {
    if ($(event.target).is('input') || toolbarDisabled) {
      return;
    }
    if (event.keyCode === 0x1b) {
      // Escape
      if (viz.scene.mode !== EditMode.POINTER) {
        activatePointer();
      } else {
        viz.scene.activeObject.set(null);
      }
    } else if (event.keyCode === 0x2e) {
      // Delete
      viz.scene.activeObject.remove();
    } else if (event.keyCode === 0x71) {
      // F2
      activateStation();
    } else if (event.altKey) {
      if (event.keyCode === 0x30) {
        // Alt+0
        // activateRuler();
      } else if (event.keyCode === 0x31) {
        // Alt+1
        activateConnectorStraight();
      } else if (event.keyCode === 0x32) {
        // Alt+2
        activateConnectorRightAngled();
      } else if (event.keyCode === 0x33) {
        // Alt+3
        activateConnectorSCurved();
      } else if (event.keyCode === 0x36) {
        // Alt+6
        activateLocHintZone();
      } else if (event.keyCode === 0x37) {
        // Alt+7
        activateNoRotateZone();
      }
    }
  }
  $(document.body).on('keyup', onKeyupAO);

  connectors.hide();
  connectorStraight.css('display', 'inline-flex');
  activatePointer();

  /* Group 3: Connected Components */

  /* Group 4: Undo, Redo & Clear Everything */
  var utilities = toolbar.find('.toolbar-utilities');
  popup(utilities[0], {
    event: 'click',
    target: 'toolbar-utilities-popup',
    placement: 'bottom'
  });
  var undo = toolbar.find('.toolbar-undo').on('click', () => viz.models.undo());
  var redo = toolbar.find('.toolbar-redo').on('click', () => viz.models.redo());
  var clearAll = toolbar.find('.toolbar-clear-all').on('click', () => viz.models.clearAll());

  $viz.on('models.undoBuffer', function () {
    undo.attr('disabled', viz.models.canUndo() ? null : 'disabled');
    redo.attr('disabled', viz.models.canRedo() ? null : 'disabled');
  });

  function onKeyup(event) {
    if ($(event.target).is('input') || toolbarDisabled) {
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

  /* Group 5: Zooming */
  toolbar.find('.toolbar-zoom-in').on('click', function () {
    viz.zoom.zoomIn();
  });
  toolbar.find('.toolbar-zoom-out').on('click', function () {
    viz.zoom.zoomOut();
  });
  toolbar.find('.toolbar-zoom-fit').on('click', function () {
    viz.zoom.zoomFit();
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
    changesetView.hideFullscreen();
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
    changesetView.showFullscreen();

    fullscreenHide.insertAfter(fullscreenShow);
    fullscreenHide.show();
    fullscreenShow.hide();
    viz.resized();
  }

  fullscreenHide.hide();

  /* Group 6: Selection Properties */
  var junctionProperties = toolbar.find('.toolbar-junction-properties');
  var pathProperties = toolbar.find('.toolbar-path-properties');
  var stationProperties = toolbar.find('.toolbar-station-properties');
  var noRotateZoneProperties = toolbar.find('.toolbar-no-rotate-zone-properties');
  var locHintZoneProperties = toolbar.find('.toolbar-loc-hint-zone-properties');
  var multiProperties = toolbar.find('.toolbar-multi-properties');

  var junctionRfid = toolbar.find('#toolbar-junction-rfid').on('change', function () {
    viz.scene.activeObject.setJunctionRfid(junctionRfid.val());
  });
  var junctionRfidField = junctionRfid.parent();

  toolbar.find('.toolbar-path-flow').on('click', viz.scene.activeObject.togglePathFlow);
  toolbar.find('.toolbar-path-facing').on('click', viz.scene.activeObject.togglePathFacing);
  var pathBend = toolbar
    .find('.toolbar-path-bend')
    .on('click', viz.scene.activeObject.togglePathBend);
  var pathSpeed = toolbar.find('.toolbar-path-speed').on('change', function () {
    var val = parseFloat(pathSpeed.val());
    if (!Number.isFinite(val)) {
      return;
    } else if (val < pathSpeed[0].min) {
      val = parseFloat(pathSpeed[0].min);
    } else if (val > pathSpeed[0].max) {
      val = parseFloat(pathSpeed[0].max);
    }
    viz.scene.activeObject.setPathSpeed(val);
    pathSpeed.val(val);
  });
  var pathSpeedParam = toolbar.find('.toolbar-path-speed-param').on('change', function () {
    viz.scene.activeObject.setPathSpeed(pathSpeedParam.val());
  });
  var pathSpeedInherit = toolbar.find('.toolbar-path-speed-inherit').on('change', function () {
    var inherit = pathSpeedInherit.is(':checked');
    var val = inherit ? '${Unlimited}' : parseFloat(pathSpeed.val()) || 0.3;
    viz.scene.activeObject.setPathSpeed(val);
    activatePathSpeed(val);
  });
  var pathRemove = toolbar
    .find('.toolbar-path-remove')
    .on('click', viz.scene.activeObject.remove);

  var stationName = toolbar.find('#toolbar-station-name').on('change', function () {
    viz.scene.activeObject.setStationName(stationName.val());
  });
  var stationNameField = stationName.parent();
  toolbar
    .find('.toolbar-station-direction')
    .on('click', viz.scene.activeObject.toggleStationDirection);
  var stationNextOverlapped = toolbar
    .find('.toolbar-station-next-overlapped')
    .on('click', viz.scene.activeObject.nextOverlappedStation);
  var stationFindAllReferences = toolbar
    .find('.toolbar-station-find-all-references')
    .on('click', function () {
      const station = stationName.val();
      get(search)({
        label: station,
        value: '_Station_' + station
      });
    });
  var stationRemove = toolbar
    .find('.toolbar-station-remove')
    .on('click', viz.scene.activeObject.remove);
  var noRotateZoneRemove = toolbar
    .find('.toolbar-no-rotate-zone-remove')
    .on('click', viz.scene.activeObject.remove);
  var locHintZoneName = toolbar.find('#toolbar-loc-hint-zone-name').on('change', function () {
    viz.scene.activeObject.setLocHintZoneName(locHintZoneName.val());
  });
  var locHintZoneRemove = toolbar
    .find('.toolbar-loc-hint-zone-remove')
    .on('click', viz.scene.activeObject.remove);
  var multiRemove = toolbar
    .find('.toolbar-multi-remove')
    .on('click', viz.scene.activeObject.remove);

  $viz.on('models.ready', function () {
    var params = viz.models.params();
    pathSpeedParam.html('');
    for (let p of params) {
      if (p.type === 'double') {
        let option = $('<option>');
        let ref = '${' + p.name + '}';
        option.val(ref).text(ref);
        pathSpeedParam.append(option);
      }
    }
  });

  function activatePathSpeed(speed) {
    pathSpeed.attr('type', toolbarDisabled ? 'text' : 'number');
    pathSpeed.attr('placeholder', speed === null ? '#.##' : '');
    if (_.isString(speed) && speed.indexOf('${') === 0) {
      if (toolbarDisabled) {
        pathSpeed.show().val(speed);
        pathSpeedParam.hide();
      } else {
        pathSpeed.hide();
        pathSpeedParam.show().val(speed);
      }
      pathSpeedInherit.prop('checked', true);
    } else {
      pathSpeed.show().val(speed);
      pathSpeedParam.hide();
      pathSpeedInherit.prop('checked', false);
    }
  }

  $viz.on('scene.preActiveObject', function () {
    toolbar.find('input').trigger('blur');
  });

  $viz.on('scene.activeObject', function () {
    var ob = viz.scene.activeObject.get();
    var obs = viz.scene.activeObject.getAll();

    junctionProperties.hide();
    pathProperties.hide();
    stationProperties.hide();
    noRotateZoneProperties.hide();
    locHintZoneProperties.hide();
    multiProperties.hide();

    if (ob instanceof Junction) {
      junctionRfidField.show();
      junctionRfid.val(ob.rfid);
      junctionProperties.show();
    }
    if (obs.some((o) => o instanceof Path)) {
      pathSpeed.val(null);
      activatePathSpeed(viz.scene.activeObject.getPathSpeed());
      if (ob) {
        pathRemove
          .show()
          .attr('disabled', viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled');
        pathBend.show().attr('disabled', ob.shape === Path.Shape.STRAIGHT ? 'disabled' : null);
      } else {
        pathRemove.hide();
        pathBend.hide();
      }
      pathProperties.show();
    }
    if (obs.some((o) => o instanceof Station)) {
      if (ob) {
        stationName.val(ob.name);
        stationRemove
          .show()
          .attr('disabled', viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled');
        stationNameField.show();
        stationNextOverlapped.show();
      } else {
        stationRemove.hide();
        stationNameField.hide();
        stationNextOverlapped.hide();
      }
      stationProperties.show();
    }
    if (ob instanceof NoRotateZone) {
      noRotateZoneRemove.attr(
        'disabled',
        viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled'
      );
      noRotateZoneProperties.show();
    }
    if (ob instanceof LocHintZone) {
      locHintZoneName.val(ob.name);
      locHintZoneRemove.attr(
        'disabled',
        viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled'
      );
      locHintZoneProperties.show();
    }
    if (obs && obs.length > 1) {
      multiRemove.attr(
        'disabled',
        viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled'
      );
      multiProperties.show();
    }
  });

  junctionProperties.hide();
  pathProperties.hide();
  stationProperties.hide();
  noRotateZoneProperties.hide();
  locHintZoneProperties.hide();
  multiProperties.hide();

  /* Group 7: Misc */
  var searchTools = toolbar.find('.toolbar-search-tools');
  var searchField = toolbar
    .find('#toolbar-search-tools-field')
    .on('change', executeSearch)
    .on('focusout', hideSearchTools);
  var searchList = toolbar.find('#toolbar-search-tools-list');

  function showSearchTools() {
    viz.scene.activeObject.set(null);
    searchTools.show();
    searchField.trigger('focus');
    refreshSearchList();
  }

  function hideSearchTools() {
    searchTools.hide();
    searchField.val('');
  }

  $viz.on('models.ready', function () {
    let search = viz.models.rawSearch();
    if (!search) {
      return;
    }

    // Allow render first.
    window.setTimeout(function () {
      searchField.val(search);
      showSearchTools();
      searchField.trigger('change');
    }, 500);
  });

  function refreshSearchList() {
    searchList.empty();
    var stations = viz.models.rawStations();

    for (let station of _.sortBy(stations, 'name')) {
      var option = $('<option>');
      option.val(station.name);
      // store ob directly will lose its class type (cash-dom issue?)
      option.data('type', 'station');
      searchList.append(option);
    }
  }

  function executeSearch() {
    var val = searchField.val();

    if (val.startsWith('_jid_')) {
      return executeSearchJunction(val);
    }

    executeSearchStation(val);
  }

  function executeSearchJunction(val) {
    var junctions = viz.models.rawJunctions();
    var jid = parseInt(val.slice(5));
    if (jid < 0 || jid >= junctions.length) {
      return;
    }
    var junction = junctions[jid];
    viz.scene.activeObject.set(junction, true);
    searchField.trigger('blur');
  }

  function executeSearchStation(label) {
    var stations = viz.models.rawStations();
    var st = stations.filter((s) => s.name === label);
    if (st.length > 0) {
      viz.scene.activeObject.set(st[0], true);
      searchField.trigger('blur');
    }
  }

  function onKeydown(event) {
    if (event.ctrlKey && event.keyCode === 0x46) {
      // Ctrl + F
      showSearchTools();
      return false;
    }
  }
  $(document.body).on('keydown', onKeydown);
  hideSearchTools();

  /* Final */
  var toolbarDisabled = false;
  var toolbarDisabledElements = [
    pointer,
    station,
    connectors,
    connectorMenu,
    noRotateZone,
    locHintZone,
    ruler,
    clearAll,
    utilities
  ];
  var toolbarDisabledElements2 = [
    junctionProperties.find('*'),
    pathProperties.find('*'),
    stationProperties.find('*'),
    noRotateZoneProperties.find('*'),
    locHintZoneProperties.find('*'),
    multiProperties.find('*')
  ];

  function disable() {
    activatePointer();
    for (let elem of toolbarDisabledElements) {
      elem.addClass('disabled');
    }
    for (let elem of toolbarDisabledElements2) {
      elem.addClass('disabled').attr('disabled', 'disabled');
    }
    toolbarDisabled = true;
  }

  function enable() {
    for (let elem of toolbarDisabledElements) {
      elem.removeClass('disabled');
    }
    for (let elem of toolbarDisabledElements2) {
      elem.removeClass('disabled').removeAttr('disabled');
    }
    toolbarDisabled = false;
  }

  return {
    destroy() {
      $(document.body).off('keyup', onKeyupAO);
      $(document.body).off('keyup', onKeyup);
      $(document.body).off('keydown', onKeydown);
    }
  };
}
