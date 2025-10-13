/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import { popup } from '@skeletonlabs/skeleton';

import bottomPanelF from 'mapx-layout-editor/toolbar/bottom-panel';
import EditMode from '../scene/edit-mode';

var dynamicPathSvg = `<path d="M 14 0 C 1 7,20 7,1 16" stroke-dasharray="3 2"></path>`;
var lineIconSvg = `<line x1="0" y1="1" x2="10" y2="13"></line>`;
var rectangleIconSvg = `<rect x="0.75" y="2.75" width="8.5" height="8.5"></rect>`;
var whiteRectangleIconSvg = `
<rect x="0.25" y="2.25" width="9.5" height="9.5" style="stroke-width:2.0; stroke-dasharray:2"></rect>
`;
var whiteRectangle2IconSvg = `
<rect x="0.25" y="2.25" width="9.5" height="9.5" style="stroke-width:2.0; stroke-dasharray:2"></rect>
<path d="M 2 7 C 4 4,6 10,8 7"></path>
`;
var clearRectangleIconSvg = `
<rect x="0.25" y="2.25" width="9.5" height="9.5" style="stroke-width:2.0; stroke-dasharray:2"></rect>
<polygon points="2 4,5 4,6.5 7,3.5 7" style="stroke-width:0.5; fill:black"></polygon>
<polygon points="3.5 7,6.5 7,8 10,5 10" style="stroke-width:0.5"></polygon>
`;

/* Toolbar template */
var toolbarHtml =
  `
<div class="agv05-toolbar">
` +
  /* Group 1: Save, Upload & Download */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none !variant-filled-primary toolbar-save" tip-title="Save" disabled>
    <i class="fa fa-floppy-disk"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-upload" tip-title="Upload">
    <i class="fa fa-upload"></i>
  </button>
  <input type="file" accept=".png" style="display:none"></input>
  <button type="button" class="btn-icon rounded-none toolbar-download" tip-title="Download">
    <i class="fa fa-download"></i>
  </button>
</div>
` +
  /* Group 2: Line, Rectangle, White Rectangle, White Rectangle 2, Clear Rectangle & Ruler */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-pointer active" tip-title="Pointer (Esc)">
    <i class="fa fa-mouse-pointer"></i>
  </button>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-draw toolbar-line" tip-title="Draw Black Line (Alt+1)">
    <i><svg width="10" height="14" class="stroke-[black]">${lineIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-draw toolbar-rectangle" tip-title="Draw Black Hollow Rectangle (Alt+2)">
    <i><svg width="10" height="14" class="icon stroke-[black] fill-none">${rectangleIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-draw-menu">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-draw-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn toolbar-draw-option toolbar-line w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black]">${lineIconSvg}</svg></span>
      <span class="flex-auto">Draw Black Line (Alt+1)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-draw-option toolbar-rectangle w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-[black] fill-none">${rectangleIconSvg}</svg></span>
      <span class="flex-auto">Draw Black Hollow Rectangle (Alt+2)</span>
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-draw-white toolbar-white-rectangle" tip-title="Draw White Rectangle (Alt+3)">
    <i><svg width="10" height="14" class="stroke-black fill-white">${whiteRectangleIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-draw-white toolbar-white-rectangle-2" tip-title="Draw White Rectangle Behind Black Pixels (Alt+4)">
    <i><svg width="10" height="14" class="stroke-black fill-white">${whiteRectangle2IconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-draw-white toolbar-clear-rectangle" tip-title="Erase (Alt+5)">
    <i><svg width="10" height="14" class="stroke-black fill-white">${clearRectangleIconSvg}</svg></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-draw-white-menu">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-draw-white-menu-popup">
  <ul class="bg-surface-200-700-token rounded">
    <li><button type="button" class="btn toolbar-white-rectangle toolbar-draw-white-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-black fill-white">${whiteRectangleIconSvg}</svg></span>
      <span class="flex-auto">Draw White Rectangle (Alt+3)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-white-rectangle-2 toolbar-draw-white-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-black fill-white">${whiteRectangle2IconSvg}</svg></span>
      <span class="flex-auto">Draw White Rectangle Behind Black Pixels (Alt+4)</span>
    </button></li>
    <li><button type="button" class="btn toolbar-clear-rectangle toolbar-draw-white-option w-full">
      <span class="badge"><svg width="10" height="14" class="stroke-black fill-white">${clearRectangleIconSvg}</svg></span>
      <span class="flex-auto">Erase (Alt+5)</span>
    </button></li>
  </ul>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-ruler" tip-title="Measure Distance (Alt+0)">
    <span>&#x1F4CF;&#xFE0E;</span>
  </button>
</div>
` +
  /* Group 3: Rotation & Resizing */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-rotate" tip-title="Rotate">
    <i class="fa fa-rotate-right fa-rotate-90"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-resize" tip-title="Resize Canvas">
    <i class="fa fa-crop"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-darken" tip-title="Darken">
    <i class="fa fa-adjust"></i>
  </button>
</div>
` +
  /* Group 4: Undo & Redo */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-utilities">
    <i class="fa fa-ellipsis-h"></i>
  </button>
</div>
<div class="z-50 bg-gray-400 border border-gray-400 rounded" data-popup="toolbar-utilities-popup">
  <button type="button" class="btn-icon rounded-none toolbar-undo" disabled="disabled" tip-title="Undo (Ctrl+Z)">
    <i class="fa-solid fa-rotate-left"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-redo" disabled="disabled" tip-title="Redo (Ctrl+Y)">
    <i class="fa-solid fa-rotate-right"></i>
  </button>
</div>
` +
  /* Group 5: Show Grid & Zooming */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-show-grid" tip-title="Show / Hide Grid (Alt+G)">
    <span>&#x25a6;</span>
  </button>
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
  <span class="badge text-base rounded-none live-connected disabled" tip-title="Connected">
    <i class="fa fa-chain"></i>
  </span>
  <span class="badge text-base rounded-none live-disconnected disabled" tip-title="Disconnected">
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
    <button type="button" class="btn-icon rounded-none live-go-to" tip-title="Go To Position">
      <i class="fa fa-location-arrow"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-reverse-go-to" tip-title="Reverse Go To Position">
      <i class="fa fa-location-arrow fa-rotate-180"></i>
    </button>
    <button type="button" class="btn-icon rounded-none live-go-to-dynamic" tip-title="Dynamic Go To Position">
      <span class="relative">
        <i><svg width="14" height="20" class="stroke-[black] fill-none">${dynamicPathSvg}</svg></i>
        <i class="fa-solid fa-location-arrow absolute -left-2 -bottom-2"></i>
      </span>
    </button>
    <button type="button" class="btn-icon rounded-none live-reverse-go-to-dynamic" tip-title="Dynamic Reverse Go To Position">
      <span class="relative">
        <i><svg width="14" height="20" class="stroke-[black] fill-none">${dynamicPathSvg}</svg></i>
        <i class="fa-solid fa-location-arrow fa-rotate-180 absolute -left-1 -bottom-1"></i>
      </span>
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
    <li><button type="button" class="btn live-apply-wall w-full">
      <span class="flex-auto">Apply Detected Wall</span>
    </button></li>
  </ul>
</div>
` +
  /* Group 8: Selection properties */
  `
<span class="divider-vertical toolbar-metadata"></span>
<div class="inline-block toolbar-metadata">Resize Canvas: </div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded h-[43px] border-none">
    <div class="input-group-shim text-black">dX <i class="fa fa-arrows-v"></i></div>
    <input type="number" id="toolbar-metadata-width" min="1" max="1000" step="0.05" tip-title="X dimension (in meters)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded h-[43px] border-none">
    <div class="input-group-shim text-black">dY <i class="fa fa-arrows-h"></i></div>
    <input type="number" id="toolbar-metadata-height" min="1" max="1000" step="0.05" tip-title="Y dimension (in meters)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded h-[43px] border-none">
    <div class="input-group-shim text-black">Resolution</div>
    <input type="number" id="toolbar-metadata-resolution" min="0.01" max="1" step="0.001" readonly="readonly" tip-title="Resolution (in meter/pixel)"></input>
  </div>
</div>
<div class="inline-block toolbar-metadata">Canvas Offset: </div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded h-[43px] border-none">
    <div class="input-group-shim text-black">X <i class="fa fa-long-arrow-up"></i></div>
    <input type="number" id="toolbar-metadata-x0" min="-1000" max="1000" step="0.001" tip-title="X offset (in meters)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Y <i class="fa fa-long-arrow-left"></i></div>
    <input type="number" id="toolbar-metadata-y0" min="-1000" max="1000" step="0.001" tip-title="Y offset (in meters)"></input>
  </div>
</div>
<div class="inline-block toolbar-metadata">Raw Map Offset: </div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">X <i class="fa fa-long-arrow-up"></i></div>
    <input type="number" id="toolbar-metadata-x-shift" min="-1000" max="1000" step="0.05" tip-title="X offset within canvas (in meters)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-metadata">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Y <i class="fa fa-long-arrow-left"></i></div>
    <input type="number" id="toolbar-metadata-y-shift" min="-1000" max="1000" step="0.05" tip-title="Y offset within canvas (in meters)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-metadata">
  <button type="button" class="btn rounded-none toolbar-metadata-apply !variant-filled-primary">
    <i class="fa fa-check"></i>
    Apply
  </button>
</div>
<div class="btn-group rounded toolbar-metadata">
  <button type="button" class="btn rounded-none toolbar-metadata-cancel">
    <i class="fa fa-times"></i>
    Cancel
  </button>
</div>
` +
  /* Group 9: Rotation Properties */
  `
<span class="divider-vertical toolbar-rotation-group"></span>
<div class="inline-block toolbar-rotation-group">Rotation: </div>
<div class="btn-group rounded toolbar-rotation-group">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Step </div>
        <div>
          <input type="number" id="toolbar-rotation-step" min="0.1" max="360" step="0.001" tip-title="Step (in degrees)"></input>
        </div>
  </div>
</div>
<div class="btn-group rounded toolbar-rotation-group">
  <button type="button" class="btn-icon rounded-none toolbar-rotation-rotate-anticlockwise" tip-title="Rotate Anticlockwise">
    <i class="fa fa-rotate-left fa-rotate-270"></i>
  </button>
</div>
<div class="btn-group rounded toolbar-rotation-group">
  <button type="button" class="btn-icon rounded-none toolbar-rotation-rotate-clockwise" tip-title="Rotate Clockwise">
    <i class="fa fa-rotate-right fa-rotate-90"></i>
  </button>
</div>
<div class="btn-group rounded toolbar-rotation-group">
  <button type="button" class="btn rounded-none toolbar-rotation-apply !variant-filled-primary" tip-title="Apply">
    <i class="fa fa-check"></i>
    Apply
  </button>
</div>
<div class="btn-group rounded toolbar-rotation-group">
  <button type="button" class="btn rounded-none toolbar-rotation-cancel" tip-title="Cancel">
    <i class="fa fa-times"></i>
    Cancel
  </button>
</div>
` +
  /* Group 10: Darken Properties */
  `
<span class="divider-vertical toolbar-darken-group"></span>
<div class="inline-block toolbar-darken-group">Darken: </div>
<div class="btn-group rounded toolbar-darken-group">
  <div class="input-group grid-cols-[auto_1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Treshold </div>
    <input type="number" id="toolbar-darken-treshold" min="0" max="255" step="1" tip-title="Darken treshold (0 - 255)"></input>
  </div>
</div>
<div class="btn-group rounded toolbar-darken-group">
  <button type="button" class="btn rounded-none toolbar-darken-apply !variant-filled-primary" tip-title="Apply">
    <i class="fa fa-check"></i>
    Apply
  </button>
</div>
<div class="btn-group rounded toolbar-darken-group">
  <button type="button" class="btn rounded-none toolbar-darken-cancel" tip-title="Cancel">
    <i class="fa fa-times"></i>
    Cancel
  </button>
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
  toolbar.insertBefore(viz.node());
  var titleInfo = $viz.parent().siblings('.title-info');

  /* Group 1: Save, Upload & Download */
  var save = toolbar.find('.toolbar-save').on('click', function () {
    hideFullscreen();
    viz.editor.save();
  });
  var upload = toolbar.find('.toolbar-upload').on('click', function () {
    uploader.trigger('click');
  });
  var uploader = upload.next().on('change', function () {
    if (this.files && this.files[0]) {
      if (this.files[0].size > 10 * 1024 * 1024) {
        window.alert('Please select image smaller than 10MB.');
        return;
      }
      var reader = new FileReader();
      reader.onload = function (e) {
        var png = new Image();
        png.onload = function () {
          viz.models.uploadPng(png);
        };
        png.onerror = function () {
          window.alert('Invalid image format.');
        };
        png.src = e.target.result;
      };
      reader.readAsDataURL(this.files[0]);
    }
    // clear file input so that the change event is fired even
    // when the user re-uploads the same file.
    uploader.val('');
  });
  var download = toolbar.find('.toolbar-download').on('click', function () {
    const link = document.createElement('a');
    let url = viz.models.downloadPng();
    link.href = url;
    link.download = 'raw.png';
    document.body.appendChild(link);
    link.click();
    setTimeout(() => {
      window.URL.revokeObjectURL(url);
      document.body.removeChild(link);
    }, 150);
  });

  $viz.on('models.dirty', function () {
    save.removeAttr('disabled');
  });

  /* Group 2: Line, Rectangle, White Rectangle, White Rectangle 2, Clear Rectangle & Ruler */
  var pointer = toolbar.find('.toolbar-pointer').on('click', activatePointer);
  var draw = toolbar.find('.toolbar-draw');
  var drawMenu = toolbar.find('.toolbar-draw-menu');
  var drawOptions = toolbar.find('.toolbar-draw-option');
  popup(drawMenu[0], {
    event: 'click',
    target: 'toolbar-draw-menu-popup',
    placement: 'bottom'
  });
  var line = toolbar.find('.toolbar-line').on('click', activateLine);
  var rectangle = toolbar.find('.toolbar-rectangle').on('click', activateRectangle);
  var drawWhite = toolbar.find('.toolbar-draw-white');
  var drawWhiteMenu = toolbar.find('.toolbar-draw-white-menu');
  var drawWhiteOptions = toolbar.find('.toolbar-draw-white-option');
  popup(drawWhiteMenu[0], {
    event: 'click',
    target: 'toolbar-draw-white-menu-popup',
    placement: 'bottom'
  });
  var whiteRectangle = toolbar
    .find('.toolbar-white-rectangle')
    .on('click', activateWhiteRectangle);
  var whiteRectangle2 = toolbar
    .find('.toolbar-white-rectangle-2')
    .on('click', activateWhiteRectangle2);
  var clearRectangle = toolbar
    .find('.toolbar-clear-rectangle')
    .on('click', activateClearRectangle);
  var ruler = toolbar.find('.toolbar-ruler').on('click', activateRuler);

  function activatePointer() {
    pointer.addClass('active');
    draw.removeClass('active');
    drawWhite.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.POINTER);
    viz.zoom.enable();
  }

  function activateLine() {
    drawOptions.removeClass('active');
    draw.hide();
    line.css('display', 'inline-flex');
    line.addClass('active');
    pointer.removeClass('active');
    drawWhite.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.LINE);
    viz.zoom.disablePan();
  }

  function activateRectangle() {
    drawOptions.removeClass('active');
    draw.hide();
    rectangle.css('display', 'inline-flex');
    rectangle.addClass('active');
    pointer.removeClass('active');
    drawWhite.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.RECTANGLE);
    viz.zoom.disablePan();
  }

  function activateWhiteRectangle() {
    drawWhiteOptions.removeClass('active');
    drawWhite.hide();
    whiteRectangle.css('display', 'inline-flex');
    whiteRectangle.addClass('active');
    pointer.removeClass('active');
    draw.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.WHITE_RECTANGLE);
    viz.zoom.disablePan();
  }

  function activateWhiteRectangle2() {
    drawWhiteOptions.removeClass('active');
    drawWhite.hide();
    whiteRectangle2.css('display', 'inline-flex');
    whiteRectangle2.addClass('active');
    pointer.removeClass('active');
    draw.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.WHITE_RECTANGLE2);
    viz.zoom.disablePan();
  }

  function activateClearRectangle() {
    drawWhiteOptions.removeClass('active');
    drawWhite.hide();
    clearRectangle.css('display', 'inline-flex');
    clearRectangle.addClass('active');
    pointer.removeClass('active');
    draw.removeClass('active');
    ruler.removeClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.CLEAR_RECTANGLE);
    viz.zoom.disablePan();
  }

  function activateRuler() {
    pointer.removeClass('active');
    draw.removeClass('active');
    drawWhite.removeClass('active');
    ruler.addClass('active');
    disableOthers();
    viz.scene.setEditMode(EditMode.RULER);
    viz.zoom.disablePan();
  }

  function disableAll() {
    pointer.removeClass('active');
    draw.removeClass('active');
    drawWhite.removeClass('active');
    ruler.removeClass('active');
  }

  $viz.on('scene.editMode', function () {
    if (viz.scene.mode === EditMode.POINTER) {
      activatePointer();
    }
  });

  function onKeyupMode(event) {
    if ($(event.target).is('input') || toolbarDisabled) {
      return;
    }
    if (event.keyCode === 0x1b) {
      // Escape
      activatePointer();
    } else if (event.altKey) {
      if (event.keyCode === 0x30) {
        // Alt+0
        activateRuler();
      } else if (event.keyCode === 0x31) {
        // Alt+1
        activateLine();
      } else if (event.keyCode === 0x32) {
        // Alt+2
        activateRectangle();
      } else if (event.keyCode === 0x33) {
        // Alt+3
        activateWhiteRectangle();
      } else if (event.keyCode === 0x34) {
        // Alt+4
        activateWhiteRectangle2();
      } else if (event.keyCode === 0x35) {
        // Alt+5
        activateClearRectangle();
      }
    }
  }
  $(document.body).on('keyup', onKeyupMode);

  draw.hide();
  line.css('display', 'inline-flex');
  drawWhite.hide();
  whiteRectangle.css('display', 'inline-flex');

  /* Group 3: Rotation & Resizing */
  var resize = toolbar.find('.toolbar-resize').on('click', editMetadata);
  var rotate = toolbar.find('.toolbar-rotate').on('click', editRotation);
  var darken = toolbar.find('.toolbar-darken').on('click', editDarken);

  /* Group 4: Undo & Redo */
  var utilities = toolbar.find('.toolbar-utilities');
  popup(utilities[0], {
    event: 'click',
    target: 'toolbar-utilities-popup',
    placement: 'bottom'
  });
  var undo = toolbar.find('.toolbar-undo').on('click', () => viz.models.undo());
  var redo = toolbar.find('.toolbar-redo').on('click', () => viz.models.redo());

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

  /* Group 5: Show Grid & Zooming */
  var showGrid = toolbar.find('.toolbar-show-grid');
  if (!viz.options.grid) {
    showGrid.remove();
  } else {
    showGrid.on('click', toggleGrid);
    viz.scene.hideGrid();
  }

  function toggleGrid() {
    if (showGrid.hasClass('active')) {
      viz.scene.hideGrid();
      showGrid.removeClass('active');
    } else {
      viz.scene.showGrid();
      showGrid.addClass('active');
    }
  }

  function onKeyupGrid(event) {
    if ($(event.target).is('input')) {
      return;
    }
    if (event.altKey) {
      if (event.keyCode === 0x47) {
        // Alt+G
        toggleGrid();
      }
    }
  }
  if (viz.options.grid) {
    $(document.body).on('keyup', onKeyupGrid);
  }

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
    activateLiveSetPose();
    updateLiveApp();
  });

  var resetMenu = toolbar.find('.live-reset-menu');
  popup(resetMenu[0], {
    event: 'click',
    target: 'live-reset-menu-popup',
    placement: 'bottom'
  });
  var liveManualSetPose = toolbar.find('.live-manual-set-pose');
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
  var liveGoTo = toolbar.find('.live-go-to').on('click', function () {
    activateLiveGoTo();
    updateLiveApp();
  });

  var liveReverseGoTo = toolbar.find('.live-reverse-go-to').on('click', function () {
    activateLiveGoTo(true);
    updateLiveApp();
  });

  var liveDynamicGoTo = toolbar.find('.live-go-to-dynamic').on('click', function () {
    activateLiveGoTo(false, true);
    updateLiveApp();
  });

  var liveDynamicReverseGoTo = toolbar
    .find('.live-reverse-go-to-dynamic')
    .on('click', function () {
      activateLiveGoTo(true, true);
      updateLiveApp();
    });

  if (!viz.options.dynamic) {
    liveDynamicGoTo.hide();
    liveDynamicReverseGoTo.hide();
  }

  var liveAbort = toolbar.find('.live-abort-action').on('click', function () {
    viz.liveApp.abortAction();
  });

  var liveApplyMenu = toolbar.find('.live-apply-menu');
  popup(liveApplyMenu[0], {
    event: 'click',
    target: 'live-apply-menu-popup',
    placement: 'bottom'
  });

  var liveApplyWall = toolbar.find('.live-apply-wall').on('click', function () {
    let spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: { content: 'Waiting for laser data' }
    };
    if (viz.modalStore) {
      viz.modalStore.trigger(spinner);
    }
    viz.liveApp.applyWall(function (error) {
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

  function activateLiveSetPose() {
    liveSetPose.addClass('active');
    liveManualSetPose.removeClass('active');
    liveGoTo.removeClass('active');
    liveReverseGoTo.removeClass('active');
    liveDynamicGoTo.removeClass('active');
    liveDynamicReverseGoTo.removeClass('active');
    disableAll();
    viz.scene.setEditMode(EditMode.LIVE_SET_POSE);
    viz.zoom.disablePan();
  }

  function activateLiveManualSetPose() {
    liveManualSetPose.addClass('active');
    liveSetPose.removeClass('active');
    liveGoTo.removeClass('active');
    liveReverseGoTo.removeClass('active');
    liveDynamicGoTo.removeClass('active');
    liveDynamicReverseGoTo.removeClass('active');
    disableAll();
    viz.scene.setEditMode(EditMode.LIVE_MANUAL_SET_POSE);
    viz.liveApp.startPoseCorrection();
  }

  function activateLiveGoTo(reverse = false, dynamic = false) {
    if (dynamic && !viz.options.dynamic) {
      return;
    }
    liveGoTo.removeClass('active');
    liveReverseGoTo.removeClass('active');
    liveDynamicGoTo.removeClass('active');
    liveDynamicReverseGoTo.removeClass('active');
    if (dynamic && !reverse) {
      liveDynamicGoTo.addClass('active');
    } else if (dynamic && reverse) {
      liveDynamicReverseGoTo.addClass('active');
    } else if (!reverse) {
      liveGoTo.addClass('active');
    } else {
      liveReverseGoTo.addClass('active');
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
  }

  function disableOthers() {
    liveSetPose.removeClass('active');
    liveManualSetPose.removeClass('active');
    liveGoTo.removeClass('active');
    liveReverseGoTo.removeClass('active');
    liveDynamicGoTo.removeClass('active');
    liveDynamicReverseGoTo.removeClass('active');
  }

  function updateLiveApp() {
    var isLiveConnected = viz.liveApp.connected();
    liveConnected.toggleClass('hidden', !isLiveConnected);
    liveDisconnected.toggleClass('hidden', isLiveConnected);

    liveSetPose.prop('disabled', true);
    liveManualSetPose.prop('disabled', true);
    liveGoTo.prop('disabled', true);
    liveReverseGoTo.prop('disabled', true);
    liveDynamicGoTo.prop('disabled', true);
    liveDynamicReverseGoTo.prop('disabled', true);
    liveAbort.prop('disabled', true);
    liveApplyWall.prop('disabled', true);

    if (!isLiveConnected) {
      return;
    }

    liveSetPose.prop('disabled', !viz.liveApp.canSetPose());
    liveManualSetPose.prop('disabled', !viz.liveApp.canSetPose());
    liveGoTo.prop('disabled', !viz.liveApp.canGoToPose());
    liveReverseGoTo.prop('disabled', !viz.liveApp.canGoToPose());
    liveDynamicGoTo.prop('disabled', !viz.liveApp.canGoToPose());
    liveDynamicReverseGoTo.prop('disabled', !viz.liveApp.canGoToPose());
    liveAbort.prop('disabled', !viz.liveApp.canAbortAction());
    liveApplyWall.prop('disabled', !viz.liveApp.canApplyWall());
  }

  $viz.on('live.updated', updateLiveApp);
  liveGroup.hide();
  updateLiveApp();

  /* Group 8: Selection Properties */
  var metadata;
  var metadataClone; // preserve original metadata, in case user cancel operation
  var metadataGroup = toolbar.find('.toolbar-metadata');
  var metadataWidth = toolbar.find('#toolbar-metadata-width').on('change', function () {
    var val = _getNumericInput(metadataWidth);
    if (val === false) {
      return;
    }
    metadata.width = Math.round(val / metadata.resolution);
    viz.modelsUpdated();
  });
  var metadataHeight = toolbar.find('#toolbar-metadata-height').on('change', function () {
    var val = _getNumericInput(metadataHeight);
    if (val === false) {
      return;
    }
    metadata.height = Math.round(val / metadata.resolution);
    viz.modelsUpdated();
  });
  var metadataResolution = toolbar.find('#toolbar-metadata-resolution');
  var metadataX0 = toolbar.find('#toolbar-metadata-x0').on('change', function () {
    var val = _getNumericInput(metadataX0);
    if (val === false) {
      return;
    }
    metadata.x0 = val;
    viz.modelsUpdated();
  });
  var metadataY0 = toolbar.find('#toolbar-metadata-y0').on('change', function () {
    var val = _getNumericInput(metadataY0);
    if (val === false) {
      return;
    }
    metadata.y0 = val;
    viz.modelsUpdated();
  });
  var metadataXShift = toolbar.find('#toolbar-metadata-x-shift').on('change', function () {
    var val = _getNumericInput(metadataXShift);
    if (val === false) {
      return;
    }
    metadata.x_shift = Math.round(val / metadata.resolution);
    viz.modelsUpdated();
  });
  var metadataYShift = toolbar.find('#toolbar-metadata-y-shift').on('change', function () {
    var val = _getNumericInput(metadataYShift);
    if (val === false) {
      return;
    }
    metadata.y_shift = Math.round(val / metadata.resolution);
    viz.modelsUpdated();
  });
  toolbar.find('.toolbar-metadata-apply').on('click', applyMetadata);
  var metadataCancel = toolbar.find('.toolbar-metadata-cancel').on('click', cancelMetadata);

  function _getNumericInput(element) {
    var val = parseFloat(element.val());
    if (!Number.isFinite(val)) {
      return false;
    } else if (val < element[0].min) {
      val = parseFloat(element[0].min);
    } else if (val > element[0].max) {
      val = parseFloat(element[0].max);
    }
    element.val(val);
    return val;
  }

  function editMetadata(event, png) {
    metadata = viz.models.metadata();
    metadataClone = Object.assign({}, metadata);
    if (png) {
      // temporary png that will be rendered by canvas
      metadata.png = png;
      viz.modelsUpdated();
    }
    var res = metadata.resolution;
    metadataWidth.attr('step', res).val(metadata.width * res);
    metadataHeight.attr('step', res).val(metadata.height * res);
    metadataResolution.val(res);
    metadataX0.val(metadata.x0);
    metadataY0.val(metadata.y0);
    metadataXShift.attr('step', res).val(metadata.x_shift * res);
    metadataYShift.attr('step', res).val(metadata.y_shift * res);
    metadataCancel.show();
    metadataGroup.show();
    disable();
    viz.zoom.zoomFit();
  }

  function applyMetadata() {
    var metadataNew = Object.assign({}, metadata);
    Object.assign(metadata, metadataClone); // restore old metadata to be pushed onto undo buffer.
    viz.models.updateMetadata(metadataNew, viz.canvas.node());
    metadataGroup.hide();
    enable();
  }

  function cancelMetadata() {
    Object.assign(metadata, metadataClone);
    viz.modelsUpdated();
    metadataGroup.hide();
    enable();
  }

  $viz.on('models.metadataMismatch', function (event, png) {
    editMetadata(event, png);
    metadataCancel.hide();
  });

  metadataGroup.hide();

  /* Group 9: Rotation Properties */
  var canvasOverlay;
  var canvasOverlayCtx;
  var rotationDeg = 0;
  var rotationGroup = toolbar.find('.toolbar-rotation-group');
  var rotationStep = toolbar.find('#toolbar-rotation-step');

  toolbar.find('.toolbar-rotation-rotate-clockwise').on('click', function () {
    var step = _getNumericInput(rotationStep);
    if (step === false) {
      return;
    }

    rotationDeg -= step; // clockwise
    rotateCanvas(rotationDeg);
  });

  toolbar.find('.toolbar-rotation-rotate-anticlockwise').on('click', function () {
    var step = _getNumericInput(rotationStep);
    if (step === false) {
      return;
    }

    rotationDeg += step; // anticlockwise
    rotateCanvas(rotationDeg);
  });

  toolbar.find('.toolbar-rotation-apply').on('click', applyRotation);
  toolbar.find('.toolbar-rotation-cancel').on('click', cancelRotation);

  function editRotation() {
    metadata = viz.models.metadata();
    metadataClone = Object.assign({}, metadata);

    canvasOverlay = document.createElement('canvas');
    canvasOverlayCtx = canvasOverlay.getContext('2d', { willReadFrequently: true });

    canvasOverlay.width = metadata.width;
    canvasOverlay.height = metadata.height;
    canvasOverlayCtx.drawImage(viz.models.png(), 0, 0);

    metadata.png = canvasOverlay;

    rotationDeg = 0;
    rotationStep.val(10);

    rotationGroup.show();
    disable();
    viz.zoom.zoomFit();
  }

  function rotateCanvas(rotationDeg) {
    // Resizing Canvas for Rotation
    var canvasRect = calcRotatedSize(rotationDeg, metadataClone.width, metadataClone.height);

    if (canvasRect.width > metadataClone.width) {
      metadata.width = canvasRect.width;
    }
    if (canvasRect.height > metadataClone.height) {
      metadata.height = canvasRect.height;
    }

    var dx = metadataClone.width - metadata.width;
    var dy = metadataClone.height - metadata.height;
    metadata.x0 = metadataClone.x0 + (dx * metadata.resolution) / 2;
    metadata.y0 = metadataClone.y0 + (dy * metadata.resolution) / 2;

    canvasOverlayCtx.save();
    canvasOverlay.width = metadata.width;
    canvasOverlay.height = metadata.height;
    canvasOverlayCtx.clearRect(0, 0, metadata.width, metadata.height);
    canvasOverlayCtx.translate(metadata.width / 2, metadata.height / 2);
    canvasOverlayCtx.rotate((rotationDeg * Math.PI) / 180);
    canvasOverlayCtx.drawImage(
      viz.models.png(),
      -metadataClone.width / 2,
      -metadataClone.height / 2
    );
    _recolorPixels(50);
    canvasOverlayCtx.restore();

    // Update canvas via metadata
    viz.modelsUpdated();
  }

  function applyRotation() {
    var metadataNew = Object.assign({}, metadata);
    Object.assign(metadata, metadataClone); // restore old metadata to be pushed onto undo buffer.
    viz.models.updateMetadata(metadataNew, viz.canvas.node());
    canvasOverlay = null;
    canvasOverlayCtx = null;
    rotationGroup.hide();
    enable();
  }

  function cancelRotation() {
    Object.assign(metadata, metadataClone);
    viz.modelsUpdated();
    canvasOverlay = null;
    canvasOverlayCtx = null;
    rotationGroup.hide();
    enable();
  }

  function calcRotatedSize(deg, x, y) {
    var r = ((deg + 90) / 180) * Math.PI;
    var width = Math.abs(x * Math.sin(r)) + Math.abs(y * Math.cos(r));
    var height = Math.abs(x * Math.cos(r)) + Math.abs(y * Math.sin(r));
    return {
      width: width,
      height: height
    };
  }

  // function to blacken gray pixels
  function _recolorPixels(treshold) {
    var imageData = canvasOverlayCtx.getImageData(
      0,
      0,
      canvasOverlay.width,
      canvasOverlay.height
    );

    var data = new Uint32Array(imageData.data.buffer); // divide into 32 bits per index (to isolate each pixels)
    var data8 = new Uint8Array(imageData.data.buffer); // divide into 8 bits per index (to isolate RGBA values)
    var idx = 0;
    var i, j;
    var a, b, g, r;

    const A_THRESHOLD = 50;

    for (i = 0; i < imageData.height; ++i) {
      for (j = 0; j < imageData.width; ++j) {
        r = data8[4 * idx + 0];
        g = data8[4 * idx + 1];
        b = data8[4 * idx + 2];
        a = data8[4 * idx + 3];
        if (a > A_THRESHOLD && r < treshold && g < treshold && b < treshold) {
          // if the pixel is visible AND the pixelvalue is black enough
          data[idx] = 0xff000000; // black (in format AABBGGRR)
        }
        ++idx;
      }
    }

    canvasOverlayCtx.putImageData(imageData, 0, 0);
  }

  rotationGroup.hide();

  /* Group 10: Darken Properties */
  var darkenGroup = toolbar.find('.toolbar-darken-group');
  var darkenTreshold = toolbar.find('#toolbar-darken-treshold').on('change', darkenChanged);
  toolbar.find('.toolbar-darken-apply').on('click', applyDarken);
  toolbar.find('.toolbar-darken-cancel').on('click', cancelDarken);

  function editDarken() {
    metadata = viz.models.metadata();

    canvasOverlay = document.createElement('canvas');
    canvasOverlayCtx = canvasOverlay.getContext('2d', { willReadFrequently: true });

    canvasOverlay.width = metadata.width;
    canvasOverlay.height = metadata.height;
    canvasOverlayCtx.drawImage(viz.models.png(), 0, 0);

    metadata.png = canvasOverlay;

    darkenTreshold.val(0);

    darkenGroup.show();
    disable();
    viz.zoom.zoomFit();
  }

  function darkenChanged() {
    var treshold = _getNumericInput(darkenTreshold);
    if (treshold === false) {
      return;
    }
    canvasOverlayCtx.clearRect(0, 0, metadata.width, metadata.height);
    canvasOverlayCtx.drawImage(viz.models.png(), 0, 0);
    _recolorPixels(treshold);
    viz.modelsUpdated();
  }

  function applyDarken() {
    metadata.png = null;
    viz.models.updatePng(viz.canvas.node());
    canvasOverlay = null;
    canvasOverlayCtx = null;
    darkenGroup.hide();
    enable();
  }

  function cancelDarken() {
    metadata.png = null;
    viz.modelsUpdated();
    canvasOverlay = null;
    canvasOverlayCtx = null;
    darkenGroup.hide();
    enable();
  }

  darkenGroup.hide();

  /* Final */
  var toolbarDisabled = false;
  var toolbarDisabledElements = [
    save,
    upload,
    download,
    pointer,
    draw,
    drawMenu,
    drawWhite,
    drawWhiteMenu,
    ruler,
    rotate,
    darken,
    resize,
    utilities
  ];

  function disable() {
    bottomPanel.disable();
    activatePointer();
    for (let elem of toolbarDisabledElements) {
      elem.addClass('disabled');
    }
    toolbarDisabled = true;
  }

  function enable() {
    bottomPanel.enable();
    for (let elem of toolbarDisabledElements) {
      elem.removeClass('disabled');
    }
    if (!viz.models.isDirty()) {
      save.addClass('disabled');
    }
    toolbarDisabled = false;
  }

  // additional panel
  var bottomPanel = bottomPanelF(viz);

  activatePointer();

  return {
    destroy() {
      destroyed = true;
      bottomPanel.destroy();
      $(document.body).off('keyup', onKeyupMode);
      $(document.body).off('keyup', onKeyup);
      $(document.body).off('keyup', onKeyupGrid);
      if (_correcting) {
        clearInterval(_correcting);
        _correcting = null;
      }
    }
  };
}
