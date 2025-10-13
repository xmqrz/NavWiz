/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import Junction from '../models/junction';
import Path from '../models/path';
import Station from '../models/station';
import Layer from '../models/layer';

var layerSvg = `
<g style="fill:dimgrey;stroke:none">
	<path
		 d="M 0.33932996,3.9604462 9.8303262,0 20,3.6364891 10.757044,7.844462 Z" />
	<path
		 d="M 17.002467,6.1008803 10.913568,8.8729365 3.4980028,6.1079486 0.2481278,7.464186 10.665641,11.348094 19.908785,7.1400227 Z" />
	<path
		 d="M 16.754339,9.7527859 10.665442,12.524842 3.249875,9.7598543 0,11.116092 10.417509,15 19.660653,10.791928 Z" />
</g>
`;

/* right panel template */
var rightPanelHtml = `
<div class="agv05x-toolbar-right">
  <div>
    <div class="btn-group-vertical" role="group" aria-label="...">
      <button type="button" class="btn btn-default toolbar-layer-enable" data-toggle="tooltip" data-placement="bottom" tip-title="Layer Panel"
        style="background:transparent;border:none">
        <svg width="20" height="15" class="icon">${layerSvg}</svg>
      </button>
    </div>
  </div>
  <div class="card toolbar-layer-panel hidden shadow-xl">
    <div class="card-header flex">
      Layer Panel
      <button type="button" class="toolbar-layer-disable text-right flex-grow"><span aria-hidden="true">&times;</span></button>
    </div>
    <form>
      <div class="p-5 pb-0">
        <select class="select toolbar-layer-active">
          <option value="-1" disabled>----------</option>
        </select>
      </div>
      <div class="p-5">
        <span>Layer In View</span>
        <div class="layer-selection"></div>
      </div>
    </form>
  </div>
</div>
`;

var visibleLayerOptHtml = (name, val, className) => `
<div>
  <label class="${className} m-0 py-2 px-5">
    <input checked type="checkbox" name="visible_layer_opt" class="checkbox mr-2" value="${val}">
    ${name}
  </label>
</div>
`;

export default function (viz) {
  var $viz = $(viz.node());
  var showLayer = false;

  var rightPanel = $(rightPanelHtml);
  var activeLayerSel = rightPanel.find('.toolbar-layer-active');
  var layerPanel = rightPanel.find('.toolbar-layer-panel');
  var enableLayerPanel = rightPanel.find('.toolbar-layer-enable').on('click', function () {
    if (showLayer) {
      return;
    }
    showLayer = true;
    enableLayerPanel.css('display', 'none');
    layerPanel.css('display', 'block');
  });
  var disableLayerPanel = rightPanel.find('.toolbar-layer-disable').on('click', function () {
    if (!showLayer) {
      return;
    }
    showLayer = false;
    enableLayerPanel.css('display', 'flex');
    layerPanel.css('display', 'none');
  });
  var visibleLayersSel = rightPanel.find('.layer-selection');
  Layer.list.forEach(([key, val]) => {
    let className = _.toLower(key);
    let displayName = _.startCase(className);
    let opt = $('<option>');
    opt.text(displayName);
    opt.val(val);
    activeLayerSel.append(opt);

    let cb = visibleLayerOptHtml(displayName, val, className);
    visibleLayersSel.append(cb);
  });

  activeLayerSel.on('change', function () {
    let layer = parseInt(activeLayerSel.val());
    visibleLayersSel.find(`input[value="${layer}"]`).prop('checked', true);
    viz.models.setActiveLayer(layer);
    viz.scene.activeObject.setLayer(layer);
    viz.modelsUpdated();
  });

  visibleLayersSel.on('change', function (e) {
    let opt = $(e.target);
    let layer = parseInt(opt.val());
    let activeLayer = viz.models.rawActiveLayer();
    if (activeLayer === layer && !opt.prop('checked')) {
      let newActiveLayer = -1;
      for (const sel of visibleLayersSel.find('input:checked')) {
        let val = parseInt(sel.value);
        if (val !== layer) {
          newActiveLayer = val;
          break;
        }
      }

      if (newActiveLayer === -1) {
        opt.prop('checked', true);
        return;
      }
      activeLayerSel.val(newActiveLayer);
      viz.models.setActiveLayer(newActiveLayer);
    }
    viz.models.setVisibleLayer(layer, opt.prop('checked'));
    viz.modelsUpdated();

    if (opt.prop('checked')) {
      return;
    }
    // remove hidden ob from activeObject.
    let obs = viz.scene.activeObject.getAll();
    viz.scene.activeObject.set(
      obs.filter((ob) => {
        if (ob instanceof Junction || ob instanceof Path || ob instanceof Station) {
          return ob.layer !== layer;
        }
        return true;
      })
    );
  });

  $viz.on('scene.activeObject', function () {
    let obs = viz.scene.activeObject.getAll();
    let newLayer = -1;
    if (obs.length === 0) {
      newLayer = viz.models.rawActiveLayer();
    } else {
      for (let ob of obs) {
        if (ob.layer >= 0) {
          if (newLayer === -1) {
            newLayer = ob.layer;
          } else if (newLayer !== ob.layer) {
            newLayer = -1;
            break;
          }
        }
      }
    }

    activeLayerSel.val(newLayer);
  });

  // Fullscreen
  $viz.on('showFullscreen', function () {
    rightPanel.addClass('agv05-fullscreen');
  });
  $viz.on('hideFullscreen', function () {
    rightPanel.removeClass('agv05-fullscreen');
  });

  rightPanel.insertBefore($viz);

  function disable() {
    disableLayerPanel.trigger('click');
    visibleLayersSel.find('input').prop('checked', true);
    Layer.list.forEach((d) => {
      viz.models.setVisibleLayer(d[1], true);
    });
    viz.modelsUpdated();
  }

  function enable() {}

  return {
    disable: disable,
    enable: enable
  };
}
