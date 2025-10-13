/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import * as d3 from 'd3';

import defF from 'mapx-layout-editor/defs';
import dimensionViz from './viz';

var divHtml = `
<div>
<ul class="flex border-b" role="tablist" id="dimensionProfileTabs">
  <li role="presentation">
    <button type="button" id="agv-body-tab" class="block py-2 px-4 font-semibold hover:text-blue-800 rounded-xl bg-surface-400" role="tab">Body</button>
  </li>
`;

for (let i = 1; i <= 5; i++) {
  divHtml += `
<li role="presentation">
  <button type="button" id="payload${i}-tab" class="block py-2 px-4 font-semibold hover:text-blue-500 rounded-xl" role="tab">Payload ${i}</button>
</li>`;
}

divHtml += `
</ul>
<section id="agv05-dimension-pre-defs" class="agv05-dimension" style="width: 100px; height: 0; overflow: hidden">
  <svg class="viz map-viz rounded-xl"></svg>
</section>
<div class="clearfix">&nbsp;</div>
<div class="tab-content p-4"></div>
</div>
`;

var bodyPanelHtml = `
<div role="tabpanel" class="grid grid-cols-2 tab-pane active" id="agv-body">
<div class="space-y-2">
  <div id="div_id_body_template" class="grid grid-cols-6">
    <label class="label font-medium col-span-2">Template</label>
    <div class="col-span-4">
      <select id="id_body_template" class="select form-control rounded-token" required>
        <option value="zalpha">Zalpha</option>
        <option value="zetha">Zetha</option>
        <option value="titan">Titan</option>
        <option value="custom">Custom SVG</option>
      </select>
    </div>
  </div>
  <div id="div_id_body_variation" class="grid grid-cols-2">
    <label class="label font-medium">Variation</label>
    <div>
      <select id="id_body_variation" class="select form-control rounded-token" required>
        <option value="standard">Standard</option>
        <option value="extended">Extended</option>
        <option value="custom" disabled="">Custom</option>
      </select>
    </div>
  </div>
  <div id="div_id_body_length" class="grid grid-cols-2">
    <label class="label font-medium">Length</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_body_length" type="number" step="0.001" min="0.01" max="3.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_body_width" class="grid grid-cols-2">
    <label class="label font-medium">Width</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_body_width" type="number" step="0.001" min="0.01" max="3.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_body_vcenter" class="grid grid-cols-2">
    <label class="label font-medium">Center of Rotation</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_body_vcenter" type="number" step="0.001" min="0.01" max="3.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_body_svg_path" class="grid grid-cols-2">
    <label class="label font-medium">SVG Path</label>
    <div>
      <textarea id="id_body_svg_path" class="textarea form-control" style="resize:vertical" rows="10" required></textarea>
    </div>
  </div>
  <div id="div_id_body_safety_margin" class="grid grid-cols-3">
    <label class="label font-medium">Safety Margin</label>
    <div class="grid grid-cols-1 xl:grid-cols-2 text-sm col-span-2">
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Top</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Bottom</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Left</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Right</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
    </div>
  </div>
</div>
<div class="agv05-dimension px-4">
  <svg class="viz map-viz rounded-xl"></svg>
</div>
<div class="clearfix">&nbsp;</div>
</div>
`;

var payloadPanelHtml = (i) => `
<div role="tabpanel" class="grid grid-cols-2 tab-pane" id="payload${i}">
<div class="space-y-2">
  <div id="div_id_payload${i}_template" class="grid grid-cols-2">
    <label class="label font-medium">Template</label>
    <div>
      <select id="id_payload${i}_template" class="select form-control rounded-token" required>
        <option value="trolley">Trolley</option>
        <option value="pallet">Pallet</option>
        <option value="custom">Custom SVG</option>
      </select>
    </div>
  </div>
  <div id="div_id_payload${i}_length" class="grid grid-cols-2">
    <label class="label font-medium">Length</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_payload${i}_length" type="number" step="0.001" min="0.01" max="3.0" required>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_payload${i}_width" class="grid grid-cols-2">
    <label class="label font-medium">Width</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_payload${i}_width" type="number" step="0.001" min="0.01" max="3.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_payload${i}_vcenter" class="grid grid-cols-2">
    <label class="label font-medium">Center (Y-axis)</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_payload${i}_vcenter" type="number" step="0.001" min="-2.0" max="2.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_payload${i}_hcenter" class="grid grid-cols-2">
    <label class="label font-medium">Center (X-axis)</label>
    <div class="input-group input-group-divider grid-cols-[1fr_auto]">
      <input id="id_payload${i}_hcenter" type="number" step="0.001" min="-2.0" max="2.0" required/>
      <div class="input-group-shim">m</div>
    </div>
  </div>
  <div id="div_id_payload${i}_svg_path" class="grid grid-cols-2">
    <label class="label font-medium">SVG Path</label>
    <div>
      <textarea id="id_payload${i}_svg_path" class="textarea form-control" style="resize:vertical" rows="10" required></textarea>
    </div>
  </div>
  <div id="div_id_payload${i}_safety_margin" class="grid grid-cols-3">
    <label class="label font-medium">Safety Margin</label>
    <div class="grid grid-cols-1 xl:grid-cols-2 text-sm col-span-2">
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Top</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Bottom</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Left</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
      <div class="mt-1 input-group input-group-divider grid-cols-[90px_1fr_auto]">
        <div class="input-group-shim !justify-center">Right</div>
        <input type="number" step="0.001" min="0" max="3.0" required/>
        <div class="input-group-shim">m</div>
      </div>
    </div>
  </div>
</div>
<div class="agv05-dimension px-4">
  <svg class="viz map-viz rounded-xl"></svg>
</div>
<div class="clearfix">&nbsp;</div>
</div>
`;

export default function (outer) {
  var div = $(divHtml);
  var tabContent = div.find('.tab-content');

  /* Import svg definitions */
  var viz0 = d3.select(div.find('#agv05-dimension-pre-defs .viz')[0]);
  defF(viz0);
  div.modelPath = viz0.append('path');

  /* Body panel */
  var bodyPanel = $(bodyPanelHtml);
  tabContent.append(bodyPanel);

  // TODO: replace tab with svelte tab this req manual state handling?
  div.find('#dimensionProfileTabs button').on('click', function () {
    var tabId = $(this).attr('id').replace('-tab', '');

    // Hide all tab content
    $('.tab-content > div').hide();

    // Show the selected tab content
    $('#' + tabId).show();

    div.find('#dimensionProfileTabs button').removeClass('bg-surface-400');
    $(this).addClass('bg-surface-400');
  });

  var viz = dimensionViz(bodyPanel.find('.viz'));

  var bodyTemplate = bodyPanel.find('#id_body_template').on('change', function () {
    div.models.updateBody({
      template: bodyTemplate.val()
    });
    updateBodyPanel();
  });
  var bodyVariation = bodyPanel.find('#id_body_variation').on('change', function () {
    div.models.updateBody({
      variation: bodyVariation.val()
    });
    updateBodyPanel();
  });
  var bodyLength = bodyPanel.find('#id_body_length');
  handleInput(bodyLength, function () {
    var val = getNumberInput(bodyLength);
    if (val === false) {
      return;
    }
    div.models.updateBody({
      length: val
    });
    updateBodyPanel();
  });
  var bodyWidth = bodyPanel.find('#id_body_width');
  handleInput(bodyWidth, function () {
    var val = getNumberInput(bodyWidth);
    if (val === false) {
      return;
    }
    div.models.updateBody({
      width: val
    });
    updateBodyPanel();
  });
  var bodyVCenter = bodyPanel.find('#id_body_vcenter');
  handleInput(bodyVCenter, function () {
    var val = getNumberInput(bodyVCenter);
    if (val === false) {
      return;
    }
    div.models.updateBody({
      vcenter: val
    });
    updateBodyPanel();
  });
  var bodySvgPath = bodyPanel.find('#id_body_svg_path');
  handleInput(bodySvgPath, function () {
    div.models.updateBody({
      svgPath: bodySvgPath.val()
    });
    updateBodyPanel();
  });
  var bodySafetyMargin = bodyPanel.find('#div_id_body_safety_margin').find('input');
  handleInput(bodySafetyMargin, function () {
    var idx = bodySafetyMargin.index(this);
    var val = getNumberInput($(this));
    if (val === false) {
      return;
    }
    div.models.updateBody({
      sm: [idx, val]
    });
    updateBodyPanel();
  });

  var divTemplated = bodyPanel.find(
    '#div_id_body_variation,#div_id_body_length,#div_id_body_width,#div_id_body_vcenter'
  );
  var divCustom = bodyPanel.find('#div_id_body_svg_path');

  function updateBodyPanel() {
    var body = div.models.body();
    if (body.template !== 'custom') {
      divTemplated.show();
      divCustom.hide();
      bodyVariation.find('option[value="extended"]').toggle(body.template === 'zalpha');
    } else {
      divTemplated.hide();
      divCustom.show();
    }
    bodyTemplate.val(body.template);
    bodyVariation.val(body.variation);
    bodyLength.val(body.length);
    bodyWidth.val(body.width);
    bodyVCenter.val(body.vcenter);
    bodySvgPath.val(body.svgPath);
    bodySafetyMargin.each(function (idx) {
      this.value = body.safetyMargin[idx];
    });

    bodySvgPath[0].setCustomValidity(body.error || '');
    bodySvgPath[0].reportValidity();

    viz.modelsUpdated();
    div.trigger('body.updated');
  }

  div.on('models.loaded', function () {
    viz.models = div.models;
    updateBodyPanel();
  });

  /* Payload panel */
  for (let i = 1; i <= 5; i++) {
    createPayloadPanel(i);
  }

  function createPayloadPanel(i) {
    var payloadPanel = $(payloadPanelHtml(i));
    tabContent.append(payloadPanel);

    var pViz = dimensionViz(payloadPanel.find('.viz'), i);

    var payloadTemplate = payloadPanel
      .find(`#id_payload${i}_template`)
      .on('change', function () {
        div.models.updatePayload(i - 1, {
          template: payloadTemplate.val()
        });
        updatePayloadPanel();
      });
    var payloadLength = payloadPanel.find(`#id_payload${i}_length`);
    handleInput(payloadLength, function () {
      var val = getNumberInput(payloadLength);
      if (val === false) {
        return;
      }
      div.models.updatePayload(i - 1, {
        length: val
      });
      updatePayloadPanel();
    });
    var payloadWidth = payloadPanel.find(`#id_payload${i}_width`);
    handleInput(payloadWidth, function () {
      var val = getNumberInput(payloadWidth);
      if (val === false) {
        return;
      }
      div.models.updatePayload(i - 1, {
        width: val
      });
      updatePayloadPanel();
    });
    var payloadVCenter = payloadPanel.find(`#id_payload${i}_vcenter`);
    handleInput(payloadVCenter, function () {
      var val = getNumberInput(payloadVCenter);
      if (val === false) {
        return;
      }
      div.models.updatePayload(i - 1, {
        vcenter: val
      });
      updatePayloadPanel();
    });
    var payloadHCenter = payloadPanel.find(`#id_payload${i}_hcenter`);
    handleInput(payloadHCenter, function () {
      var val = getNumberInput(payloadHCenter);
      if (val === false) {
        return;
      }
      div.models.updatePayload(i - 1, {
        hcenter: val
      });
      updatePayloadPanel();
    });
    var payloadSvgPath = payloadPanel.find(`#id_payload${i}_svg_path`);
    handleInput(payloadSvgPath, function () {
      div.models.updatePayload(i - 1, {
        svgPath: payloadSvgPath.val()
      });
      updatePayloadPanel();
    });
    var payloadSafetyMargin = payloadPanel
      .find(`#div_id_payload${i}_safety_margin`)
      .find('input');
    handleInput(payloadSafetyMargin, function () {
      var idx = payloadSafetyMargin.index(this);
      var val = getNumberInput($(this));
      if (val === false) {
        return;
      }
      div.models.updatePayload(i - 1, {
        sm: [idx, val]
      });
      updatePayloadPanel();
    });

    var divTemplated = payloadPanel.find(
      `#div_id_payload${i}_length,#div_id_payload${i}_width,#div_id_payload${i}_vcenter,#div_id_payload${i}_hcenter`
    );
    var divCustom = payloadPanel.find(`#div_id_payload${i}_svg_path`);

    function updatePayloadPanel() {
      var payload = div.models.payload(i - 1);
      if (payload.template !== 'custom') {
        divTemplated.show();
        divCustom.hide();
      } else {
        divTemplated.hide();
        divCustom.show();
      }
      payloadTemplate.val(payload.template);
      payloadLength.val(payload.length);
      payloadWidth.val(payload.width);
      payloadVCenter.val(payload.vcenter);
      payloadHCenter.val(payload.hcenter);
      payloadSvgPath.val(payload.svgPath);
      payloadSafetyMargin.each(function (idx) {
        this.value = payload.safetyMargin[idx];
      });

      payloadSvgPath[0].setCustomValidity(payload.error || '');
      payloadSvgPath[0].reportValidity();

      pViz.modelsUpdated();
    }

    div.on('body.updated', function () {
      pViz.models = div.models;
      updatePayloadPanel();
    });
    div.on('resized', function () {
      pViz.resized();
    });
  }

  /* Misc */
  // show invalid input on hidden tab.
  let onInvalid = _.debounce(
    function (e) {
      console.log('invalid call');
      let t = $(e.target);

      // show tab
      $('.tab-content > div').hide();
      let tabPane = t.closest('.tab-pane');
      tabPane.show();

      // switch tab button to the correct one.
      let tabId = tabPane.attr('id');
      div.find('#dimensionProfileTabs button').removeClass('bg-surface-400');
      div.find(`#${tabId}-tab`).addClass('bg-surface-400');

      setTimeout(() => {
        e.target.reportValidity();
      }, 100);
    },
    300,
    { leading: true, trailing: false }
  );
  div.find('input').on('invalid', onInvalid);

  // Hack: show hidden form for viz resized event.
  var form;
  if (outer.closest('form').hasClass('hidden')) {
    form = outer.closest('form');
    form.toggleClass('hidden', false);
  }

  outer.append(div);
  viz.resized();
  div.trigger('resized');

  // Hack: end
  if (form) {
    form.toggleClass('hidden', true);
  }

  // initially hide all tab content and show agv-body tab only
  $('.tab-content > div').hide();
  $('#agv-body').show();

  return div;
}

function handleInput(el, handler) {
  var debounced = _.debounce(handler, 1000);
  el.on('input', debounced).on('change', debounced.flush.bind(debounced));
}

function getNumberInput(el) {
  var val = parseFloat(el.val());
  if (!Number.isFinite(val)) {
    return false;
  } else if (val < el[0].min) {
    val = parseFloat(el[0].min);
  } else if (val > el[0].max) {
    val = parseFloat(el[0].max);
  }
  return val;
}
