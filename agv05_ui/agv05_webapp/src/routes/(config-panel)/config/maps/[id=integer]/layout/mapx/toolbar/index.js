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
import EditMode from '../scene/edit-mode';
import ForbiddenZone from '../models/forbidden-zone';
import Landmark from '../models/landmark';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import LocHintZone from 'map-layout-editor/models/loc-hint-zone';
import Path from '../models/path';
import Station from '../models/station';
import changesetViewF from '../changeset-view';
import rightPanelF from './right-panel';
import bottomPanelF from './bottom-panel';

var connectorStraightIconSvg = `<line x1="0" y1="1" x2="10" y2="13"></line>`;
var connectorBezierIconSvg = `<path d="M 1 0 V 5 C 1 7,9 7,9 9 V 14"></path>`;
var dynamicPathSvg = `<path d="M 14 0 C 1 7,20 7,1 16" stroke-dasharray="3 2"></path>`;
var landmarkIconSvg = `<g transform="translate(6,7)scale(75)"><circle r="0.08" fill="black" stroke="none"></circle><path d="M 0,.06 C 0,.02 .02,0 .06,0 .02,0 0,-.02 0,-.06 0,-.02 -.02,0 -.06,0 -.02,0 0,.02 0,.06 Z" fill="white" stroke="none"></path>`;
var locHintZoneIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="blue" fill-opacity="0.5" stroke="blue"></rect>`;
var forbiddenZoneIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="red" fill-opacity="0.5" stroke="red"></rect>`;
var noRotateZoneIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="yellow" fill-opacity="0.5" stroke="gold"></rect>`;

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
  /* Group 2: Connectors, Station, Landmark, Zones & Ruler */
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
  <button type="button" class="btn-icon rounded-none toolbar-connector toolbar-connector-bezier" tip-title="Bezier Path (Alt+2)">
    <i><svg width="10" height="14" class="stroke-[black] fill-none">${connectorBezierIconSvg}</svg></i>
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
    <li><button type="button" class="btn toolbar-connector-bezier toolbar-connector-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black] fill-none">${connectorBezierIconSvg}</svg></span>
      <span class="flex-auto">Bezier Path (Alt+2)</span>
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-landmark" tip-title="Landmark (Alt+9)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${landmarkIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zone toolbar-forbidden-zone" tip-title="Forbidden Zone (Alt+8)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${forbiddenZoneIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zone toolbar-no-rotate-zone" tip-title="No-Rotate Zone (Alt+7)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${noRotateZoneIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zone toolbar-loc-hint-zone" tip-title="Location Hint Zone (Alt+6)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${locHintZoneIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-zone-menu">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-zone-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn toolbar-loc-hint-zone toolbar-zone-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[white] dark:stroke-[black]">${locHintZoneIconSvg}</svg></span>
      <span class="flex-auto">Location Hint Zone (Alt+6)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-no-rotate-zone toolbar-zone-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[white] dark:stroke-[black]">${noRotateZoneIconSvg}</svg></span>
      <span class="flex-auto">No-Rotate Zone (Alt+7)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-forbidden-zone toolbar-zone-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[white] dark:stroke-[black]">${forbiddenZoneIconSvg}</svg></span>
      <span class="flex-auto">Forbidden Zone (Alt+8)</span>
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-ruler" tip-title="Measure Distance (Alt+0)">
    <span>&#x1F4CF;&#xFE0E;</span>
  </button>
</div>
` +
  /* Todo: Group 3: Connected Components */
  `
<!-- <div class="btn-group rounded"> -->
<!--   <button type="button" class="btn-icon rounded-none toolbar-connected-components" tip-title="Connected Components"> -->
<!--     <i class="fa fa-object-ungroup"></i> -->
<!--   </button> -->
<!-- </div> -->
  ` +
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
  /* Group 6: Apps */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-live-connect" tip-title="Connect Agv">
    <i class="fa fa-train"></i>
  </button>
</div>
` +
  /* Group 7: App Properties */
  `
<span class="divider-vertical toolbar-live"></span>
<div class="inline-block toolbar-live">Live App: </div>
<div class="btn-group rounded toolbar-live">
  <span class="badge text-base rounded-none live-connected" tip-title="Connected">
    <i class="fa fa-chain"></i>
  </span>
  <span class="badge text-base rounded-none live-disconnected" tip-title="Disconnected">
    <i class="fa fa-chain-broken"></i>
  </span>
  <button type="button" class="btn-icon rounded-none live-close" tip-title="Disable Live App">
    <i class="fa fa-times"></i>
  </button>
</div>
<div class="toolbar-live space-x-1">
  <div class="btn-group rounded">
    <button type="button" class="btn-icon rounded-none live-set-pose" tip-title="Reset Position">
      <i class="fa fa-play-circle fa-rotate-270"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-reset-menu">
      <i class="fa-solid fa-caret-down"></i>
    </button>
  </div>
  <div class="btn-group rounded">
    <button type="button" class="btn-icon rounded-none live-go-to-pose live-go-to-btn" tip-title="Go To Position">
      <i class="fa fa-location-arrow"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-reverse-go-to-pose live-go-to-btn" tip-title="Reverse Go To Position">
      <i class="fa fa-location-arrow fa-rotate-180"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-go-to-pose-dynamic live-go-to-btn" tip-title="Dynamic Go To Position">
      <span class="relative">
        <i><svg width="14" height="20" class="stroke-[black] fill-none">${dynamicPathSvg}</svg></i>
        <i class="fa-solid fa-location-arrow absolute -left-2 -bottom-2"></i>
      </span>
    </button>
    <button type="button" class="btn-icon rounded-none live-reverse-go-to-pose-dynamic live-go-to-btn" tip-title="Dynamic Reverse Go To Position">
      <span class="relative">
        <i><svg width="14" height="20" class="stroke-[black] fill-none">${dynamicPathSvg}</svg></i>
        <i class="fa-solid fa-location-arrow fa-rotate-180 absolute -left-1 -bottom-1"></i>
      </span>
    </button>
    <button type="button" class="btn-icon rounded-none live-goto-menu">
      <i class="fa-solid fa-caret-down"></i>
    </button>
  </div>
  <div class="btn-group rounded">
    <button type="button" class="btn-icon rounded-none live-nav-to" tip-title="Navigate To">
      <i class="fa fa-road"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-nav-menu">
      <i class="fa-solid fa-caret-down"></i>
    </button>
  </div>
  <div class="btn-group rounded">
    <button type="button" class="btn rounded-none live-apply-menu">
      <i class="fa-solid fa-pencil pr-2"></i>
      <i class="fa-solid fa-caret-down"></i>
    </button>
  </div>
  <div class="btn-group rounded">
    <button type="button" class="btn-icon rounded-none live-abort-action" tip-title="Abort Action">
      <i class="fa fa-circle-xmark"></i>
    </button>
  </div>
</div>
<div class="z-50" data-popup="live-reset-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn live-set-pose w-full">
      <span class="flex-auto">Reset Position</span>
    </button></li>
    <li><button type="button" class="btn live-manual-set-pose w-full">
      <span class="flex-auto">Manual Reset Position</span>
    </button></li>
    <li><button type="button" class="btn live-tracker-reset w-full">
      <span class="flex-auto">Reset Tracker to Station</span>
    </button></li>
  </ul>
</div>
<div class="z-50" data-popup="live-goto-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn live-go-to-pose live-go-to-option w-full">
      <span class="flex-auto">Go To Position</span>
    </button></li>
    <li><button type="button" class="btn live-reverse-go-to-pose live-go-to-option w-full">
      <span class="flex-auto">Reverse Go To Position</span>
    </button></li>
    <li><button type="button" class="btn live-go-to-pose-dynamic live-go-to-option w-full">
      <span class="flex-auto">Dynamic Go To Position</span>
    </button></li>
    <li><button type="button" class="btn live-reverse-go-to-pose-dynamic live-go-to-option w-full">
      <span class="flex-auto">Dynamic Reverse Go To Position</span>
    </button></li>
  </ul>
</div>
<div class="z-50" data-popup="live-nav-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn live-nav-to w-full">
      <span class="flex-auto">Navigate To</span>
    </button></li>
    <li><button type="button" class="btn live-reverse-nav-to w-full">
      <span class="flex-auto">Reverse Navigate To</span>
    </button></li>
  </ul>
</div>
<div class="z-50" data-popup="live-manual-set-pose-popup">
  <div class="flex flex-col place-content-evenly bg-secondary-400 rounded-xl p-1 relative top-[-90px] active" style="box-shadow: inset 0px 0px 5px 5px rgba(122, 117, 225, 1)">
    <div class="grid grid-cols-4">
      <button
        type="button"
        data-correction="RL"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa fa-undo"></i>
      </button>
      <button
        type="button"
        data-correction="Up"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa-solid fa-arrow-up"></i>
      </button>
      <button
        type="button"
        data-correction="RR"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa-solid fa-rotate-right"></i>
      </button>
      <button
        type="button"
        class="!variant-filled-error btn btn-sm close-popup"
        tip-title="Cancel">
        <i class="fa-solid fa-xmark"></i>
      </button>
    </div>
    <div class="grid grid-cols-4">
      <button
        type="button"
        data-correction="Left"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa-solid fa-arrow-left"></i>
      </button>
      <button
        type="button"
        data-correction="Down"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa-solid fa-arrow-down"></i>
      </button>
      <button
        type="button"
        data-correction="Right"
        class="btn btn-sm live-manual-set-pose-btn">
        <i class="fa-solid fa-arrow-right"></i>
      </button>
      <button
        type="button"
        class="!variant-filled-success btn btn-sm close-popup live-manual-set-pose-apply"
        tip-title="Update pose">
        <i class="fa-solid fa-check"></i>
      </button>
    </div>
  </div>
</div>
<div class="z-50" data-popup="live-apply-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn live-apply-reflector w-full">
      <span class="flex-auto">Apply Detected Reflector</span>
    </button></li>
  </ul>
</div>
` +
  /* Group 8: Selection properties */
  `
<span class="divider-vertical toolbar-path-properties"></span>
<div class="inline-block toolbar-path-properties">Path: </div>
<div class="btn-group rounded toolbar-path-properties">
  <button type="button" class="btn-icon rounded-none toolbar-path-flow" tip-title="Change Flow">
    <i class="fa fa-exchange"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-path-facing" tip-title="Change Forward / Reverse Motion">
    <i class="fa fa-shield fa-flip-vertical"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-path-tracked" tip-title="Switch Trackless / Dynamic / Tracked">
    <i class="fa fa-road"></i>
  </button>
</div>
<div class="btn-group rounded toolbar-path-properties">
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
<span class="divider-vertical toolbar-station-properties"></span>
<div class="inline-block toolbar-station-properties">Station: </div>
<div class="inline-flex toolbar-station-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Name</div>
    <input type="text" id="toolbar-station-name" placeholder="Name"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-station-properties">
  <button type="button" class="btn-icon rounded-none toolbar-station-heading" tip-title="Rotate Heading (Counter-clockwise)">
    <i class="fa fa-compass fa-flip-horizontal"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station-heading-reversed" tip-title="Rotate Heading (Clockwise)">
    <i class="fa fa-compass"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station-heading-quarter" tip-title="Rotate Heading 90&deg; (Counter-clockwise)">
    <i class="fa fa-puzzle-piece fa-flip-horizontal"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-station-heading-reversed-quarter" tip-title="Rotate Heading 90&deg; (Clockwise)">
    <i class="fa fa-puzzle-piece"></i>
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
<span class="divider-vertical toolbar-landmark-properties"></span>
<div class="inline-block toolbar-landmark-properties">Landmark: </div>
<div class="btn-group rounded toolbar-landmark-properties">
  <button type="button" class="btn-icon rounded-none toolbar-landmark-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical toolbar-forbidden-zone-properties"></span>
<div class="inline-block toolbar-forbidden-zone-properties">Forbidden Zone: </div>
<div class="btn-group rounded toolbar-forbidden-zone-properties">
  <button type="button" class="btn-icon rounded-none toolbar-forbidden-zone-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical toolbar-no-rotate-zone-properties"></span>
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
<span class="divider-vertical toolbar-multi-properties"></span>
<div class="inline-block toolbar-multi-properties">Sel: </div>
<div class="btn-group rounded toolbar-multi-properties">
  <button type="button" class="btn-icon rounded-none toolbar-expand-selection" tip-title="Expand Selection">
    <i class="fa fa-arrows-alt"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-multi-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
` +
  /* Group 9: Misc */
  `
<span class="divider-vertical toolbar-search-tools"></span>
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
  var destroyed = false;
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
      viz.editor.loadChangeset();
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

  /* Group 2: Connectors, Station, Landmark, Zones & Ruler */
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
  var connectorBezier = toolbar
    .find('.toolbar-connector-bezier')
    .on('click', activateConnectorBezier);
  var landmark = toolbar.find('.toolbar-landmark').on('click', activateLandmark);
  var zones = toolbar.find('.toolbar-zone');
  var zoneMenu = toolbar.find('.toolbar-zone-menu');
  var zoneOptions = toolbar.find('.toolbar-zone-option');
  popup(zoneMenu[0], {
    event: 'click',
    target: 'toolbar-zone-menu-popup',
    placement: 'bottom'
  });
  var forbiddenZone = toolbar
    .find('.toolbar-forbidden-zone')
    .css('display', viz.options.dynamic ? 'flex' : 'none')
    .on('click', activateForbiddenZone);
  var noRotateZone = toolbar.find('.toolbar-no-rotate-zone').on('click', activateNoRotateZone);
  var locHintZone = toolbar.find('.toolbar-loc-hint-zone').on('click', activateLocHintZone);
  var ruler = toolbar.find('.toolbar-ruler').on('click', activateRuler);

  function activatePointer() {
    pointer.addClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    zones.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.POINTER);
    viz.zoom.enable();
  }

  function activateStation() {
    station.addClass('active');
    pointer.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    zones.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
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
    landmark.removeClass('active');
    zones.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.CONNECTOR_STRAIGHT);
    viz.zoom.disablePan();
  }

  function activateConnectorBezier() {
    connectorOptions.removeClass('active');
    connectors.hide();
    connectorBezier.css('display', 'inline-flex');
    connectorBezier.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    landmark.removeClass('active');
    zones.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.CONNECTOR_BEZIER);
    viz.zoom.disablePan();
  }

  function activateLandmark() {
    landmark.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    zones.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.LANDMARK);
    viz.zoom.disablePan();
  }

  function activateForbiddenZone() {
    if (!viz.options.dynamic) {
      return;
    }
    zoneOptions.removeClass('active');
    zones.hide();
    forbiddenZone.css('display', 'flex');
    forbiddenZone.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.FORBIDDEN_ZONE);
    viz.zoom.disablePan();
  }

  function activateNoRotateZone() {
    zoneOptions.removeClass('active');
    zones.hide();
    noRotateZone.css('display', 'flex');
    noRotateZone.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.NO_ROTATE_ZONE);
    viz.zoom.disablePan();
  }

  function activateLocHintZone() {
    zoneOptions.removeClass('active');
    zones.hide();
    locHintZone.css('display', 'flex');
    locHintZone.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.LOC_HINT_ZONE);
    viz.zoom.disablePan();
  }

  function activateRuler() {
    ruler.addClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    zones.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.RULER);
    viz.zoom.disablePan();
  }

  function disableAll() {
    ruler.removeClass('active');
    pointer.removeClass('active');
    station.removeClass('active');
    connectors.removeClass('active');
    landmark.removeClass('active');
    zones.removeClass('active');
  }

  $viz.on('scene.editMode', function () {
    switch (viz.scene.mode) {
      case EditMode.POINTER:
        activatePointer();
        break;
      case EditMode.LANDMARK:
        activateLandmark();
        break;
      case EditMode.FORBIDDEN_ZONE:
        activateForbiddenZone();
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
      viz.scene.hoverObject.set(null); // clear aligned incase deleted
      viz.scene.activeObject.remove();
    } else if (event.keyCode === 0x71) {
      // F2
      activateStation();
    } else if (event.altKey) {
      if (event.keyCode === 0x30) {
        // Alt+0
        activateRuler();
      } else if (event.keyCode === 0x31) {
        // Alt+1
        activateConnectorStraight();
      } else if (event.keyCode === 0x32) {
        // Alt+2
        activateConnectorBezier();
      } else if (event.keyCode === 0x36) {
        // Alt+6
        activateLocHintZone();
      } else if (event.keyCode === 0x37) {
        // Alt+7
        activateNoRotateZone();
      } else if (event.keyCode === 0x38) {
        // Alt+8
        activateForbiddenZone();
      } else if (event.keyCode === 0x39) {
        // Alt+9
        activateLandmark();
      }
    }
  }
  $(document.body).on('keyup', onKeyupAO);

  connectors.hide();
  connectorStraight.css('display', 'inline-flex');
  zones.hide();
  noRotateZone.css('display', 'flex');

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
    $viz.trigger('hideFullscreen');
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
    $viz.trigger('showFullscreen');

    fullscreenHide.insertAfter(fullscreenShow);
    fullscreenHide.show();
    fullscreenShow.hide();
    viz.resized();
  }

  fullscreenHide.hide();

  /* Group 6: Apps */
  var liveApp = toolbar.find('.toolbar-live-connect').on('click', enableLiveApp);

  /* Group 7: App Properties */
  var liveGroup = toolbar.find('.toolbar-live');
  var liveConnected = toolbar.find('.live-connected');
  var liveDisconnected = toolbar.find('.live-disconnected');
  toolbar.find('.live-close').on('click', disableLiveApp);

  function enableLiveApp() {
    liveApp.attr('disabled', 'disabled');
    liveGroup.css('display', 'inline-flex');
    viz.liveApp.enable();
  }

  function disableLiveApp() {
    liveApp.removeAttr('disabled');
    liveGroup.hide();
    viz.liveApp.disable();
  }

  var liveSetPose = toolbar.find('.live-set-pose').on('click', function () {
    if (viz.liveApp.canSetStation()) {
      viz.liveApp.setStation();
      return;
    }
    activateLiveSetPose();
  });

  var liveGotoPoseBtn = toolbar.find('.live-go-to-btn');
  liveGotoPoseBtn.hide();
  var liveGotoPoseOption = toolbar.find('.live-go-to-option');
  var liveGoToPose = toolbar.find('.live-go-to-pose').on('click', function () {
    liveGotoPoseBtn.hide();
    liveGoToPose.show();
    if (viz.liveApp.canGoTo()) {
      viz.liveApp.setGoTo();
      return;
    }
    activateLiveGoTo();
  });
  liveGoToPose.show();

  var liveReverseGoToPose = toolbar.find('.live-reverse-go-to-pose').on('click', function () {
    liveGotoPoseBtn.hide();
    liveReverseGoToPose.show();
    if (viz.liveApp.canGoTo()) {
      viz.liveApp.setGoTo(true);
      return;
    }
    activateLiveGoTo(true);
  });

  var liveDynamicGoToPose = toolbar.find('.live-go-to-pose-dynamic').on('click', function () {
    liveGotoPoseBtn.hide();
    liveDynamicGoToPose.show();
    if (viz.liveApp.canGoTo()) {
      viz.liveApp.setGoTo(false, true);
      return;
    }
    activateLiveGoTo(false, true);
  });

  var liveDynamicReverseGoToPose = toolbar
    .find('.live-reverse-go-to-pose-dynamic')
    .on('click', function () {
      liveGotoPoseBtn.hide();
      liveDynamicReverseGoToPose.show();
      if (viz.liveApp.canGoTo()) {
        viz.liveApp.setGoTo(true, true);
        return;
      }
      activateLiveGoTo(true, true);
    });

  if (!viz.options.dynamic) {
    liveDynamicGoToPose.hide();
    liveDynamicReverseGoToPose.hide();
  }

  liveSetPose.setEnable = function (enable) {
    liveSetPose.prop('disabled', !enable);
  };
  liveGoToPose.setEnable = function (enable) {
    liveGoToPose.prop('disabled', !enable);
  };
  liveReverseGoToPose.setEnable = function (enable) {
    liveReverseGoToPose.prop('disabled', !enable);
  };
  liveDynamicGoToPose.setEnable = function (enable) {
    liveDynamicGoToPose.prop('disabled', !enable);
  };
  liveDynamicReverseGoToPose.setEnable = function (enable) {
    liveDynamicReverseGoToPose.prop('disabled', !enable);
  };

  var resetMenu = toolbar.find('.live-reset-menu');
  popup(resetMenu[0], {
    event: 'click',
    target: 'live-reset-menu-popup',
    placement: 'bottom'
  });
  var liveTrackerReset = toolbar.find('.live-tracker-reset').on('click', function () {
    viz.liveApp.resetTracker();
  });
  liveTrackerReset.setEnable = function (enable) {
    liveTrackerReset.prop('disabled', !enable);
  };

  var gotoMenu = toolbar.find('.live-goto-menu');
  popup(gotoMenu[0], {
    event: 'click',
    target: 'live-goto-menu-popup',
    placement: 'bottom'
  });
  var navMenu = toolbar.find('.live-nav-menu');
  popup(navMenu[0], {
    event: 'click',
    target: 'live-nav-menu-popup',
    placement: 'bottom'
  });
  var liveManualSetPose = toolbar.find('.live-manual-set-pose');
  liveManualSetPose.setEnable = function (enable) {
    liveManualSetPose.prop('disabled', !enable);
  };
  toolbar.find('.live-manual-set-pose-apply').on('click', function () {
    viz.liveApp.applyPoseCorrection(true);
  });
  popup(liveManualSetPose[0], {
    event: 'click',
    target: 'live-manual-set-pose-popup',
    placement: 'bottom',
    closeQuery: '.close-popup',
    state: onManualSetPoseState
  });
  let __liveManualSetPoseState = false;
  function onManualSetPoseState(e) {
    // TODO: bug state always being called?
    if (e.state === __liveManualSetPoseState) {
      return;
    }
    __liveManualSetPoseState = e.state;
    if (e.state) {
      activateLiveManualSetPose();
    } else {
      if (viz.scene.mode === EditMode.LIVE_MANUAL_SET_POSE) {
        viz.liveApp.applyPoseCorrection();
        if (viz.scene.mode !== EditMode.POINTER) {
          activatePointer();
        }
      }
    }
  }
  let _correcting;
  let _holding_scale = 0;
  function poseP(action) {
    // To handle race condition of this function being called after onDestroy()
    if (destroyed) {
      return;
    }
    /* button pressed */
    viz.liveApp.updatePoseCorrection(action, 1);
    _correcting = setInterval(function () {
      _holding_scale += 1;
      let correction_scale = 5;
      if (_holding_scale > 15) {
        correction_scale = 50;
      } else if (_holding_scale > 10) {
        correction_scale = 20;
      } else if (_holding_scale > 5) {
        correction_scale = 10;
      }
      viz.liveApp.updatePoseCorrection.bind(null, action, correction_scale)();
    }, 250);
  }
  function poseR() {
    /* button released */
    _holding_scale = 0;
    if (_correcting) {
      clearInterval(_correcting);
      _correcting = null;
    }
  }
  toolbar
    .find('.live-manual-set-pose-btn')
    .on('mousedown', function () {
      let d = this.getAttribute('data-correction');
      poseP(d);
    })
    .on('mouseup', function () {
      poseR();
    });
  var liveNavTo = toolbar.find('.live-nav-to').on('click', function () {
    viz.liveApp.setNavTo();
  });
  var liveReverseNavTo = toolbar.find('.live-reverse-nav-to').on('click', function () {
    viz.liveApp.setNavTo(true);
  });
  var liveAbort = toolbar.find('.live-abort-action').on('click', function () {
    viz.liveApp.abortAction();
  });
  liveNavTo.setEnable = function (enable) {
    liveNavTo.prop('disabled', !enable);
  };
  liveReverseNavTo.setEnable = function (enable) {
    liveReverseNavTo.prop('disabled', !enable);
  };
  liveAbort.setEnable = function (enable) {
    liveAbort.prop('disabled', !enable);
  };

  var liveApplyMenu = toolbar.find('.live-apply-menu');
  popup(liveApplyMenu[0], {
    event: 'click',
    target: 'live-apply-menu-popup',
    placement: 'bottom'
  });
  var liveApplyReflector = toolbar.find('.live-apply-reflector').on('click', function () {
    let spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: { content: 'Waiting for reflector data' }
    };
    if (viz.modalStore) {
      viz.modalStore.trigger(spinner);
    }
    viz.liveApp.applyReflector(function (error) {
      if (viz.modalStore) {
        viz.modalStore.close(spinner);
      }
      if (error) {
        setTimeout(() => {
          alert(error);
        }, 100);
      }
    });
  });
  liveApplyReflector.setEnable = function (enable) {
    liveApplyReflector.prop('disabled', !enable);
  };

  function activateLiveSetPose() {
    liveSetPose.addClass('active');
    liveManualSetPose.removeClass('active');
    liveGotoPoseBtn.removeClass('active');
    liveGotoPoseOption.removeClass('active');
    disableAll();
    viz.scene.setEditMode(EditMode.LIVE_SET_POSE);
    viz.zoom.disablePan();
    updateLiveApp();
  }

  function activateLiveManualSetPose() {
    liveManualSetPose.addClass('active');
    liveSetPose.removeClass('active');
    liveGotoPoseBtn.removeClass('active');
    liveGotoPoseOption.removeClass('active');
    disableAll();
    viz.scene.setEditMode(EditMode.LIVE_MANUAL_SET_POSE);
    viz.liveApp.startPoseCorrection();
  }

  function activateLiveGoTo(reverse = false, dynamic = false) {
    if (dynamic && !viz.options.dynamic) {
      return;
    }
    liveGotoPoseBtn.removeClass('active');
    liveGotoPoseOption.removeClass('active');
    if (dynamic && !reverse) {
      liveDynamicGoToPose.addClass('active');
    } else if (dynamic && reverse) {
      liveDynamicReverseGoToPose.addClass('active');
    } else if (!reverse) {
      liveGoToPose.addClass('active');
    } else {
      liveReverseGoToPose.addClass('active');
    }
    liveSetPose.removeClass('active');
    liveManualSetPose.removeClass('active');
    disableAll();
    if (dynamic) {
      viz.scene.setEditMode(
        reverse ? EditMode.LIVE_DYNAMIC_REVERSE_GOTO_POSE : EditMode.LIVE_DYNAMIC_GOTO_POSE
      );
    } else {
      viz.scene.setEditMode(
        reverse ? EditMode.LIVE_REVERSE_GOTO_POSE : EditMode.LIVE_GOTO_POSE
      );
    }
    viz.zoom.disablePan();
    updateLiveApp();
  }

  function disableOthers() {
    liveSetPose.removeClass('active');
    liveManualSetPose.removeClass('active');
    liveGotoPoseBtn.removeClass('active');
    liveGotoPoseOption.removeClass('active');
  }

  function updateLiveApp() {
    var isLiveConnected = viz.liveApp.connected();
    liveConnected.toggleClass('hidden', !isLiveConnected);
    liveDisconnected.toggleClass('hidden', isLiveConnected);

    liveSetPose.setEnable(false);
    liveManualSetPose.setEnable(false);
    liveGoToPose.setEnable(false);
    liveReverseGoToPose.setEnable(false);
    liveDynamicGoToPose.setEnable(false);
    liveDynamicReverseGoToPose.setEnable(false);
    liveNavTo.setEnable(false);
    liveReverseNavTo.setEnable(false);
    liveTrackerReset.setEnable(false);
    liveAbort.setEnable(false);
    liveApplyReflector.setEnable(false);

    if (!isLiveConnected) {
      return;
    }

    liveSetPose.setEnable(viz.liveApp.canSetPose());
    liveManualSetPose.setEnable(viz.liveApp.canSetPose());
    liveGoToPose.setEnable(viz.liveApp.canGoToPose());
    liveReverseGoToPose.setEnable(viz.liveApp.canGoToPose());
    liveDynamicGoToPose.setEnable(viz.liveApp.canGoToPose());
    liveDynamicReverseGoToPose.setEnable(viz.liveApp.canGoToPose());
    liveAbort.setEnable(viz.liveApp.canAbortAction());
    liveNavTo.setEnable(viz.liveApp.canNavTo());
    liveReverseNavTo.setEnable(viz.liveApp.canNavTo());
    liveTrackerReset.setEnable(viz.liveApp.isPoseValid() && viz.liveApp.canSetStation());
    liveApplyReflector.setEnable(viz.liveApp.canApplyReflector());
  }

  $viz.on('scene.activeObject', updateLiveApp);
  $viz.on('live.updated', updateLiveApp);
  liveGroup.hide();
  updateLiveApp();

  /* Group 8: Selection Properties */
  var pathProperties = toolbar.find('.toolbar-path-properties');
  var stationProperties = toolbar.find('.toolbar-station-properties');
  var landmarkProperties = toolbar.find('.toolbar-landmark-properties');
  var forbiddenZoneProperties = toolbar.find('.toolbar-forbidden-zone-properties');
  var noRotateZoneProperties = toolbar.find('.toolbar-no-rotate-zone-properties');
  var locHintZoneProperties = toolbar.find('.toolbar-loc-hint-zone-properties');
  var multiProperties = toolbar.find('.toolbar-multi-properties');

  toolbar.find('.toolbar-path-flow').on('click', viz.scene.activeObject.togglePathFlow);
  toolbar.find('.toolbar-path-facing').on('click', viz.scene.activeObject.togglePathFacing);
  var pathTracked = toolbar
    .find('.toolbar-path-tracked')
    .on('click', viz.scene.activeObject.togglePathTracked);
  if (!viz.options.dynamic) {
    pathTracked.attr('tip-title', 'Switch Trackless / Tracked');
  }
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
    .find('.toolbar-station-heading')
    .on('click', viz.scene.activeObject.toggleStationHeading);
  toolbar
    .find('.toolbar-station-heading-reversed')
    .on('click', viz.scene.activeObject.toggleStationHeadingReversed);
  toolbar
    .find('.toolbar-station-heading-quarter')
    .on('click', viz.scene.activeObject.toggleStationHeading.bind(null, true));
  toolbar
    .find('.toolbar-station-heading-reversed-quarter')
    .on('click', viz.scene.activeObject.toggleStationHeadingReversed.bind(null, true));
  var stationNextOverlapped = toolbar
    .find('.toolbar-station-next-overlapped')
    .on('click', viz.scene.activeObject.nextOverlappedStation);
  var _stationFindAllReferences = toolbar
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
  var landmarkRemove = toolbar
    .find('.toolbar-landmark-remove')
    .on('click', viz.scene.activeObject.remove);
  var forbiddenZoneRemove = toolbar
    .find('.toolbar-forbidden-zone-remove')
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
  toolbar.find('.toolbar-expand-selection').on('click', viz.scene.activeObject.expand);
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

    pathProperties.hide();
    stationProperties.hide();
    landmarkProperties.hide();
    forbiddenZoneProperties.hide();
    noRotateZoneProperties.hide();
    locHintZoneProperties.hide();
    multiProperties.hide();

    if (obs.some((o) => o instanceof Path)) {
      pathSpeed.val(null);
      activatePathSpeed(viz.scene.activeObject.getPathSpeed());
      if (ob) {
        pathRemove
          .show()
          .attr('disabled', viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled');
      } else {
        pathRemove.hide();
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
    if (ob instanceof Landmark) {
      landmarkRemove.attr(
        'disabled',
        viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled'
      );
      landmarkProperties.show();
    }
    if (ob instanceof ForbiddenZone) {
      forbiddenZoneRemove.attr(
        'disabled',
        viz.scene.activeObject.isRemovalAllowed() ? null : 'disabled'
      );
      forbiddenZoneProperties.show();
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

  pathProperties.hide();
  stationProperties.hide();
  landmarkProperties.hide();
  forbiddenZoneProperties.hide();
  noRotateZoneProperties.hide();
  locHintZoneProperties.hide();
  multiProperties.hide();

  /* Group 9: Misc */
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
      event.preventDefault();
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
    landmark,
    zones,
    zoneMenu,
    ruler,
    clearAll,
    utilities
  ];
  var toolbarDisabledElements2 = [
    pathProperties.find('*'),
    stationProperties.find('*'),
    landmarkProperties.find('*'),
    forbiddenZoneProperties.find('*'),
    noRotateZoneProperties.find('*'),
    locHintZoneProperties.find('*'),
    multiProperties.find('*')
  ];

  function disable() {
    rightPanel.disable();
    bottomPanel.disable();
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
    rightPanel.enable();
    bottomPanel.enable();
    for (let elem of toolbarDisabledElements) {
      elem.removeClass('disabled');
    }
    for (let elem of toolbarDisabledElements2) {
      elem.removeClass('disabled').removeAttr('disabled');
    }
    toolbarDisabled = false;
  }

  // additional panel
  var rightPanel = rightPanelF(viz);
  var bottomPanel = bottomPanelF(viz);

  activatePointer();
  return {
    destroy() {
      destroyed = true;
      bottomPanel.destroy();
      $(document.body).off('keyup', onKeyupAO);
      $(document.body).off('keyup', onKeyup);
      $(document.body).off('keydown', onKeydown);
      if (_correcting) {
        clearInterval(_correcting);
        _correcting = null;
      }
    }
  };
}
