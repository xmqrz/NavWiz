/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import { popup } from '@skeletonlabs/skeleton';

import EditMode from './scene/edit-mode';
import TextAnnotation from './models/text-annotation';
import PolygonAnnotation from './models/polygon-annotation';
import IconAnnotation from './models/icon-annotation';

var polygonIconSvg = `<rect x="0.25" y="2.25" width="9.5" height="9.5" fill="white" fill-opacity="0.5" stroke="black"></rect>`;

/* Toolbar template */
var toolbarHtml =
  `
<div class="agv05-toolbar">
` +
  /* Group 1: Save */
  `
<div class="btn-group rounded rounded-tl-xl">
  <button type="button" class="btn-icon rounded-none !variant-filled-primary toolbar-save" disabled tip-title="Save">
    <i class="fa fa-floppy-disk"></i>
  </button>
</div>
` +
  /* Group 2: Text Annotation, Shape Annotation, Icon Annotation & Ruler */
  `
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-pointer active" tip-title="Select (Esc)">
    <i class="fa fa-mouse-pointer"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-text-annotation" tip-title="Text Annotation (Alt+1)">
    <i class="fa fa-font"></i>
  </button>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-polygon-annotation" tip-title="Polygon Annotation (Alt+2)">
    <span class="relative">
      <i><svg width="14" height="14" class="icon">${polygonIconSvg}</svg></i>
      <i class="fa fa-mouse-pointer absolute -right-2 -bottom-3"></i>
    </span>
  </button>
</div>
<div class="btn-group rounded">
  <button type="button" class="btn-icon rounded-none toolbar-lift-icon-annotation" tip-title="Lift Icon (Al+3)">
    <i>
      <svg class="icon stroke-[black] fill-[black]" width="14" height="14">
        <g transform="translate(7,7)scale(25)scale(0.35)">
          <use href="#lift"/>
        </g>
      </svg>
    </i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-robotarm-icon-annotation" tip-title="Robot Arm Icon (Alt+4)">
    <i>
      <svg class="icon stroke-[black] fill-[black]" width="14" height="14">
        <g transform="translate(7,7)scale(25)scale(0.28)">
          <use href="#robotarm"/>
        </g>
      </svg>
    </i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-conveyor-icon-annotation" tip-title="Conveyor Icon (Alt+5)">
    <i>
      <svg class="icon stroke-[black] fill-[black]" width="14" height="14">
        <g transform="translate(7,7)scale(25)scale(0.25)">
          <use href="#conveyor"/>
        </g>
      </svg>
    </i>
  </button>
</div>
` +
  /* Todo: Group 3: Connected Components */
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
<span class="divider-vertical h-4 toolbar-text-annotation-properties"></span>
<div class="inline-block toolbar-text-annotation-properties">Text: </div>
<div class="inline-flex toolbar-text-annotation-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Font Size</div>
    <select class="toolbar-select toolbar-text-annotation-size">
      <option value="0.2">Extra Small</option>
      <option value="0.4">Small</option>
      <option value="0.6">Medium</option>
      <option value="0.8">Large</option>
      <option value="1.0">Extra Large</option>
    </select>
  </div>
</div>
<div class="btn-group rounded toolbar-text-annotation-properties">
  <button type="button" class="btn-icon rounded-none toolbar-text-annotation-btn" tip-title="Edit Text Annotation">
    <i class="fa fa-pencil-square"></i>
  </button>
  <button type="button" class="btn-icon rounded-none toolbar-text-annotation-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<div class="z-50" data-popup="toolbar-text-annotation-popup">
  <ul class="bg-surface-100-800-token rounded p-4">
    <li>
      <label class="label text-token">Text Annotation:</label>
    </li>
    <li>
      <textarea class="textarea text-token p-4 toolbar-text-annotation-textarea whitespace-pre resize" rows="5"></textarea>
    </li>
  </ul>
</div>
<span class="divider-vertical h-4 toolbar-polygon-annotation-properties"></span>
<div class="inline-block toolbar-polygon-annotation-properties">Polygon: </div>
<div class="inline-flex toolbar-polygon-annotation-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Color</div>
    <select class="toolbar-select toolbar-polygon-annotation-color">
      <option value="#f00">Red</option>
      <option value="#0f0">Green</option>
      <option value="#00f">Blue</option>
      <option value="#FFFF00">Yellow</option>
      <option value="#000">Black</option>
      <option value="#FFF">White</option>
    </select>
  </div>
</div>
<div class="inline-flex toolbar-polygon-annotation-properties">
  <div class="input-group grid-cols-[1fr_auto] rounded bg-surface-200-700-token text-black h-[43px] border-none">
    <div class="input-group-shim text-black">Opacity</div>
    <select class="toolbar-select toolbar-polygon-annotation-opacity">
      <option value="0">0</option>
      <option value="0.2">0.2</option>
      <option value="0.4">0.4</option>
      <option value="0.6">0.6</option>
      <option value="0.8">0.8</option>
      <option value="1">1</option>
    </select>
  </div>
</div>
<div class="btn-group rounded toolbar-polygon-annotation-properties">
  <button type="button" class="btn-icon rounded-none toolbar-polygon-annotation-remove" tip-title="Delete (Del)">
    <i class="fa fa-trash"></i>
  </button>
</div>
<span class="divider-vertical h-4 toolbar-icon-annotation-properties"></span>
<div class="inline-block toolbar-icon-annotation-properties">Icon: </div>
<div class="btn-group rounded toolbar-icon-annotation-properties">
  <button type="button" class="btn-icon rounded-none toolbar-icon-annotation-remove" tip-title="Delete (Del)">
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
  /* End tag */
  `
</div>
`;

export default function (viz) {
  var toolbar = $(toolbarHtml);
  var $viz = $(viz.node());
  toolbar.insertBefore($viz);
  var titleInfo = $viz.parent().siblings('.title-info');

  /* Group 1: Save & Changesets */
  var save = toolbar.find('.toolbar-save').on('click', function () {
    hideFullscreen();
    viz.editor.save();
  });

  $viz.on('models.dirty', function () {
    save.removeAttr('disabled');
  });

  /* Group 2: Text, Shapes, Icons */
  var pointer = toolbar.find('.toolbar-pointer').on('click', activatePointer);
  var draw = toolbar.find('.toolbar-draw');
  var textAnnotation = toolbar
    .find('.toolbar-text-annotation')
    .on('click', activateTextAnnotation);
  var polygonAnnotation = toolbar
    .find('.toolbar-polygon-annotation')
    .on('click', activatePolygonAnnotation);
  var liftIcon = toolbar
    .find('.toolbar-lift-icon-annotation')
    .on('click', activateLiftAnnotation);
  var robotarmIcon = toolbar
    .find('.toolbar-robotarm-icon-annotation')
    .on('click', activateRobotArmAnnotation);
  var conveyorIcon = toolbar
    .find('.toolbar-conveyor-icon-annotation')
    .on('click', activateConveyorAnnotation);

  let toolbarItems = [
    pointer,
    textAnnotation,
    polygonAnnotation,
    liftIcon,
    robotarmIcon,
    conveyorIcon
  ];

  function removeActive() {
    for (let item of toolbarItems) {
      item.removeClass('active');
    }
  }

  function activatePointer() {
    removeActive();
    pointer.addClass('active');
    viz.scene.setEditMode(EditMode.POINTER);
    viz.zoom.enable();
  }

  function activateTextAnnotation() {
    removeActive();
    textAnnotation.addClass('active');
    viz.scene.setEditMode(EditMode.TEXT);
    viz.zoom.disablePan();
  }

  function activatePolygonAnnotation() {
    removeActive();
    polygonAnnotation.addClass('active');
    viz.scene.setEditMode(EditMode.POLYGON);
    viz.zoom.disablePan();
  }

  function activateLiftAnnotation() {
    removeActive();
    liftIcon.addClass('active');
    viz.scene.setEditMode(EditMode.LIFT);
    viz.zoom.enable();
    viz.zoom.disablePan();
  }

  function activateRobotArmAnnotation() {
    removeActive();
    robotarmIcon.addClass('active');
    viz.scene.setEditMode(EditMode.ROBOTARM);
    viz.zoom.enable();
    viz.zoom.disablePan();
  }

  function activateConveyorAnnotation() {
    removeActive();
    conveyorIcon.addClass('active');
    viz.scene.setEditMode(EditMode.CONVEYOR);
    viz.zoom.enable();
    viz.zoom.disablePan();
  }

  $viz.on('scene.editMode', function () {
    switch (viz.scene.mode) {
      case EditMode.POINTER:
        activatePointer();
        break;
      case EditMode.TEXT:
        activateTextAnnotation();
        break;
      case EditMode.POLYGON:
        activatePolygonAnnotation();
        break;
      case EditMode.LIFT:
        activateLiftAnnotation();
        break;
      case EditMode.ROBOTARM:
        activateRobotArmAnnotation();
        break;
      case EditMode.CONVEYOR:
        activateConveyorAnnotation();
        break;
    }
  });

  function onKeyupMode(event) {
    if ($(event.target).is('textarea') || toolbarDisabled) {
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
    } else if (event.altKey) {
      if (event.keyCode === 0x31) {
        // Alt+1
        activateTextAnnotation();
      } else if (event.keyCode === 0x32) {
        // Alt+2
        activatePolygonAnnotation();
      } else if (event.keyCode === 0x33) {
        // Alt+3
        activateLiftAnnotation();
      } else if (event.keyCode === 0x34) {
        // Alt+4
        activateRobotArmAnnotation();
      } else if (event.keyCode === 0x35) {
        // Alt+4
        activateConveyorAnnotation();
      }
    }
  }
  $(document.body).on('keyup', onKeyupMode);

  activatePointer();
  draw.hide();

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
  toolbar.find('.toolbar-clear-all').on('click', () => viz.models.clearAll());

  $viz.on('models.undoBuffer', function () {
    undo.attr('disabled', viz.models.canUndo() ? null : 'disabled');
    redo.attr('disabled', viz.models.canRedo() ? null : 'disabled');
  });

  function onKeyup(event) {
    if ($(event.target).is('textarea') || toolbarDisabled) {
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

    fullscreenHide.insertAfter(fullscreenShow);
    fullscreenHide.show();
    fullscreenShow.hide();
    viz.resized();
  }

  fullscreenHide.hide();

  /* Group 6: Selection Properties */
  var textAnnotationProperties = toolbar.find('.toolbar-text-annotation-properties');
  toolbar.find('.toolbar-text-annotation-remove').on('click', viz.scene.activeObject.remove);
  var textAnnotationSize = toolbar
    .find('.toolbar-text-annotation-size')
    .on('change', function () {
      viz.scene.activeObject.setFontSize(parseFloat(textAnnotationSize.val()));
    });
  var textAnnotationBtn = toolbar
    .find('.toolbar-text-annotation-btn')
    .on('click', function () {
      textAnnotationTextarea.val(textAnnotationTextarea.val() || '');
    });
  popup(textAnnotationBtn[0], {
    event: 'click',
    target: 'toolbar-text-annotation-popup',
    placement: 'bottom'
  });
  var textAnnotationTextarea = toolbar
    .find('.toolbar-text-annotation-textarea')
    .on('blur', function () {
      viz.scene.activeObject.setTextAnnotationContent(textAnnotationTextarea.val());
    });
  var polygonAnnotationProperties = toolbar.find('.toolbar-polygon-annotation-properties');
  toolbar
    .find('.toolbar-polygon-annotation-remove')
    .on('click', viz.scene.activeObject.remove);
  var polygonAnnotationColor = toolbar
    .find('.toolbar-polygon-annotation-color')
    .on('change', function () {
      viz.scene.activeObject.setPolygonFillColor(polygonAnnotationColor.val());
    });
  var polygonAnnotationOpacity = toolbar
    .find('.toolbar-polygon-annotation-opacity')
    .on('change', function () {
      viz.scene.activeObject.setPolygonFillOpacity(polygonAnnotationOpacity.val());
    });
  var iconAnnotationProperties = toolbar.find('.toolbar-icon-annotation-properties');
  toolbar.find('.toolbar-icon-annotation-remove').on('click', viz.scene.activeObject.remove);
  var multiProperties = toolbar.find('.toolbar-multi-properties');
  var _multiRemove = toolbar
    .find('.toolbar-multi-remove')
    .on('click', viz.scene.activeObject.remove);

  $viz.on('scene.preActiveObject', function () {
    toolbar.find('textarea').trigger('blur');
  });

  $viz.on('scene.activeObject', function () {
    var ob = viz.scene.activeObject.get();
    var obs = viz.scene.activeObject.getAll();

    textAnnotationProperties.hide();
    polygonAnnotationProperties.hide();
    iconAnnotationProperties.hide();
    multiProperties.hide();

    if (ob instanceof TextAnnotation) {
      textAnnotationSize.val(ob.size);
      textAnnotationTextarea.val(ob.content);
      textAnnotationProperties.show();
    }

    if (ob instanceof PolygonAnnotation) {
      polygonAnnotationColor.val(ob.fill);
      polygonAnnotationOpacity.val(ob.fillOpacity);
      polygonAnnotationProperties.show();
    }

    if (ob instanceof IconAnnotation) {
      iconAnnotationProperties.show();
    }

    if (obs && obs.length > 1) {
      multiProperties.show();
    }
  });

  textAnnotationProperties.hide();
  polygonAnnotationProperties.hide();
  iconAnnotationProperties.hide();
  multiProperties.hide();

  /* Final */
  var toolbarDisabled = false;

  return {
    destroy() {
      $(document.body).off('keyup', onKeyupMode);
      $(document.body).off('keyup', onKeyup);
    }
  };
}
