/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

export default function (form) {
  var dimension = form.dimension;

  // field data
  var body =
    {}; /* attrs: template, variation, length, width, vcenter, hcenter, svgPath, safetyMargin */
  var _bodyZalpha = {};
  var _bodyZetha = {};
  var _bodyTitan = {};
  var _bodyOmni = {};
  var _bodyCustom = {};
  var payloads =
    []; /* attrs: template, length, width, vcenter, hcenter, svgPath, safetyMargin */
  var _payloadsTrolley = [];
  var _payloadsPallet = [];
  var _payloadsCustom = [];

  /* Deserialize from database */
  function load(options) {
    if (options.body && _.isObject(options.body)) {
      updateBody(options.body);
    } else {
      updateBody({});
    }
    if (options.payloads && Array.isArray(options.payloads)) {
      for (let i = 0; i < 5; i++) {
        let payload = options.payloads[i];
        if (payload && _.isObject(payload)) {
          updatePayload(i, payload);
        } else {
          updatePayload(i, {});
        }
      }
    } else {
      for (let i = 0; i < 5; i++) {
        updatePayload(i, {});
      }
    }

    dimension.trigger('models.loaded');
  }

  /* Serialize to database */
  function getDimension() {
    return {
      body: body,
      payloads: payloads
    };
  }

  /* Helper functions */
  function computeBoundingBox() {
    if (!body.template) {
      return null;
    }
    return {
      top: body.vcenter - body.length,
      bottom: body.vcenter + 0.2, // expand by 0.2 for dimension text
      left: -body.hcenter - 0.2,
      right: body.width - body.hcenter + 0.2
    };
  }

  function getBBox(svgPath) {
    dimension.modelPath.attr('d', svgPath);
    return dimension.modelPath.node().getBBox();
  }

  /* Manipulation: Body */
  function updateBody(options) {
    body.template = options.template || body.template;
    if (['zalpha', 'zetha', 'titan', 'custom'].indexOf(body.template) < 0) {
      body.template = 'zalpha';
    }

    if (body.template === 'zalpha') {
      if (
        options.length ||
        options.width ||
        options.vcenter ||
        ['extended', 'standard'].indexOf(options.variation) < 0
      ) {
        body.length = round3(Math.max(0.4, options.length || _bodyZalpha.length || 1.03));
        body.width = round3(Math.max(0.3, options.width || _bodyZalpha.width || 0.55));
        body.vcenter = round3(
          Math.min(body.length - 0.2, options.vcenter || _bodyZalpha.vcenter || 0.51)
        );

        let dims = [body.length, body.width, body.vcenter];
        if (_.isEqual(dims, dimsZalphaExtended)) {
          body.variation = 'extended';
        } else if (_.isEqual(dims, dimsZalphaStandard)) {
          body.variation = 'standard';
        } else {
          body.variation = 'custom';
        }
      } else if (options.variation === 'extended') {
        [body.length, body.width, body.vcenter] = dimsZalphaExtended;
        body.variation = 'extended';
      } else {
        [body.length, body.width, body.vcenter] = dimsZalphaStandard;
        body.variation = 'standard';
      }
      body.hcenter = body.width / 2;
      body.svgPath = zalphaTpl(body.length, body.width, body.vcenter);
      delete body.error;
      _bodyZalpha = _.clone(body);
    } else if (body.template === 'zetha') {
      if (
        options.length ||
        options.width ||
        options.vcenter ||
        'standard' !== options.variation
      ) {
        body.length = round3(Math.max(1.0, options.length || _bodyZetha.length || 1.938));
        body.width = round3(Math.max(0.68, options.width || _bodyZetha.width || 0.78));
        body.vcenter = round3(
          Math.min(body.length - 0.8, options.vcenter || _bodyZetha.vcenter || 0.652)
        );

        let dims = [body.length, body.width, body.vcenter];
        if (_.isEqual(dims, dimsZethaStandard)) {
          body.variation = 'standard';
        } else {
          body.variation = 'custom';
        }
      } else {
        [body.length, body.width, body.vcenter] = dimsZethaStandard;
        body.variation = 'standard';
      }
      body.hcenter = body.width / 2;
      body.svgPath = zethaTpl(body.length, body.width, body.vcenter);
      delete body.error;
      _bodyZetha = _.clone(body);
    } else if (body.template === 'titan') {
      if (
        options.length ||
        options.width ||
        options.vcenter ||
        'standard' !== options.variation
      ) {
        body.length = round3(Math.max(0.6, options.length || _bodyTitan.length || 1.36));
        body.width = round3(Math.max(0.5, options.width || _bodyTitan.width || 0.96));
        body.vcenter = round3(
          Math.max(
            0.25,
            Math.min(body.length - 0.3, options.vcenter || _bodyTitan.vcenter || 0.68)
          )
        );

        let dims = [body.length, body.width, body.vcenter];
        if (_.isEqual(dims, dimsTitanStandard)) {
          body.variation = 'standard';
        } else {
          body.variation = 'custom';
        }
      } else {
        [body.length, body.width, body.vcenter] = dimsTitanStandard;
        body.variation = 'standard';
      }
      body.hcenter = body.width / 2;
      body.svgPath = titanTpl(body.length, body.width, body.vcenter);
      delete body.error;
      _bodyTitan = _.clone(body);
    } else if (body.template === 'omni') {
      if (
        options.length ||
        options.width ||
        options.vcenter ||
        'standard' !== options.variation
      ) {
        body.length = round3(Math.max(1.0, options.length || _bodyOmni.length || 1.066));
        body.width = round3(Math.max(0.5, options.width || _bodyOmni.width || 0.55));
        body.vcenter = round3(
          Math.max(
            0.5,
            Math.min(body.length - 0.5, options.vcenter || _bodyOmni.vcenter || 0.533)
          )
        );

        let dims = [body.length, body.width, body.vcenter];
        if (_.isEqual(dims, dimsOmniStandard)) {
          body.variation = 'standard';
        } else {
          body.variation = 'custom';
        }
      } else {
        [body.length, body.width, body.vcenter] = dimsOmniStandard;
        body.variation = 'standard';
      }
      body.hcenter = body.width / 2;
      body.svgPath = omniTpl(body.length, body.width, body.vcenter);
      delete body.error;
      _bodyOmni = _.clone(body);
    } else if (body.template === 'custom') {
      body.variation = 'custom';
      body.svgPath = options.svgPath || _bodyCustom.svgPath || body.svgPath || '';

      let bbox = getBBox(body.svgPath);
      body.length = round3(bbox.height);
      body.width = round3(bbox.width);
      body.vcenter = round3(bbox.y + bbox.height);
      body.hcenter = round3(-bbox.x);

      if (!body.length || !body.width) {
        body.error = 'Invalid svg path.';
      } else if (body.length > 3 || body.width > 3) {
        body.error = 'The body exceeds the maximum allowable dimension.';
      } else if (
        body.vcenter <= 0 ||
        body.vcenter >= body.length ||
        body.hcenter <= 0 ||
        body.hcenter >= body.width
      ) {
        body.error = 'The body is too far from the origin (0, 0).';
      } else {
        delete body.error;
      }
      _bodyCustom = _.clone(body);
    }

    body.safetyMargin = options.safetyMargin || body.safetyMargin;
    if (!Array.isArray(body.safetyMargin) || body.safetyMargin.length !== 4) {
      body.safetyMargin = [0, 0, 0, 0];
    }
    if (options.sm) {
      let [idx, val] = options.sm;
      val = round3(Math.max(0.0, Math.min(3.0, val)));
      body.safetyMargin[idx] = val;
    }
  }

  /* Manipulation: Payload */
  function updatePayload(i, options) {
    payloads[i] = payloads[i] || {};
    var payload = payloads[i];
    payload.template = options.template || payload.template;
    if (['trolley', 'pallet', 'custom'].indexOf(payload.template) < 0) {
      payload.template = 'trolley';
    }

    if (payload.template === 'trolley') {
      let _payloadTrolley = _payloadsTrolley[i] || {};
      payload.length = round3(Math.max(0.2, options.length || _payloadTrolley.length || 0.8));
      payload.width = round3(Math.max(0.2, options.width || _payloadTrolley.width || 0.6));
      payload.vcenter = round3(
        options.vcenter !== undefined
          ? options.vcenter
          : _payloadTrolley.vcenter !== undefined
            ? _payloadTrolley.vcenter
            : 0.1
      );
      payload.hcenter = round3(
        options.hcenter !== undefined ? options.hcenter : _payloadTrolley.hcenter || 0
      );
      payload.svgPath = trolleyTpl(
        payload.length,
        payload.width,
        payload.vcenter,
        payload.hcenter
      );
      delete payload.error;
      _payloadsTrolley[i] = _.clone(payload);
    } else if (payload.template === 'pallet') {
      let _payloadPallet = _payloadsPallet[i] || {};
      payload.length = round3(Math.max(0.5, options.length || _payloadPallet.length || 1.0));
      payload.width = round3(Math.max(0.5, options.width || _payloadPallet.width || 1.0));
      payload.vcenter = round3(
        options.vcenter !== undefined ? options.vcenter : _payloadPallet.vcenter || 0
      );
      payload.hcenter = round3(
        options.hcenter !== undefined ? options.hcenter : _payloadPallet.hcenter || 0
      );
      payload.svgPath = palletTpl(
        payload.length,
        payload.width,
        payload.vcenter,
        payload.hcenter
      );
      delete payload.error;
      _payloadsPallet[i] = _.clone(payload);
    } else if (payload.template === 'custom') {
      var _payloadCustom = _payloadsCustom[i] || {};
      payload.svgPath = options.svgPath || _payloadCustom.svgPath || payload.svgPath || '';

      let bbox = getBBox(payload.svgPath);
      payload.length = round3(bbox.height);
      payload.width = round3(bbox.width);
      payload.vcenter = round3(bbox.y + bbox.height / 2);
      payload.hcenter = round3(bbox.x + bbox.width / 2);

      if (!payload.length || !payload.width) {
        payload.error = 'Invalid svg path.';
      } else if (payload.length > 3 || body.width > 3) {
        payload.error = 'The payload exceeds the maximum allowable dimension.';
      } else if (
        payload.vcenter < -2 ||
        payload.vcenter > 2 ||
        payload.hcenter < -2 ||
        payload.hcenter > 2
      ) {
        payload.error = 'The payload is too far from the origin (0, 0).';
      } else {
        delete payload.error;
      }
      _payloadsCustom[i] = _.clone(payload);
    }

    payload.safetyMargin = options.safetyMargin || payload.safetyMargin;
    if (!Array.isArray(payload.safetyMargin) || payload.safetyMargin.length !== 4) {
      payload.safetyMargin = [0, 0, 0, 0];
    }
    if (options.sm) {
      let [idx, val] = options.sm;
      val = round3(Math.max(0.0, Math.min(3.0, val)));
      payload.safetyMargin[idx] = val;
    }
  }

  dimension.models = {
    load: load,
    getDimension: getDimension,

    computeBoundingBox: computeBoundingBox,

    updateBody: updateBody,
    updatePayload: updatePayload,

    body: () => body,
    payload: (i) => payloads[i]
  };
}

/* helper function */
function zalphaTpl(length, width, vcenter) {
  // clockwise
  const x1 = -width / 2;
  const x1a = x1 + 0.12;
  const x2 = -x1;
  const x2a = -x1a;

  const y1 = -(length - vcenter);
  const y1a = y1 + 0.05;
  const y2 = vcenter;
  const y2a = y2 - 0.05;

  const wheel = '-0.07 v0.14 h-0.04 v-0.14 h0.04';

  return r4`M${x2} ${y1a} V${y2a} l-0.12 0.05 H${x1a} l-0.12 -0.05 V${y1a} l0.12 -0.05 H${x2a} l0.12 0.05 m0 0.14 H${x1} M${x2 - 0.04} ${wheel} M${x1 + 0.08} ${wheel}`;
}

function zethaTpl(length, width, vcenter) {
  // clockwise
  const x1 = -width / 2;
  const x1a = x1 + 0.045;
  const x2 = -x1;
  const x2a = -x1a;

  const y1 = -(length - vcenter);
  const y1a = y1 + 0.6645;
  const y2 = vcenter;
  const y2a = y2 - 0.1;

  const fork = r4`V${y2a} l-0.04 0.1 h-0.117 l-0.04 -0.1 V${y1a}`;
  const wheel = '-0.04 v0.08 h-0.06 v-0.08 h0.06';
  const circle = 'a0.04 0.04 0 1 1 0.08 0 a0.04 0.04 0 1 1 -0.08 0';

  return r4`M${x2a} ${y1 + 0.2} v0.4645 H0.286 ${fork} h-0.178 ${fork} H${x1a} v-0.4645 h-0.045 v-0.134 l0.07 -0.066 H${x2 - 0.07} l0.07 0.066 v0.134 h-0.045 m0 0.101 l-0.06 -0.061 H${x1a + 0.06} l-0.06 0.061 v-0.18 l0.045 -0.05 H${x2a - 0.045} l0.045 0.05 v0.18 m-0.032 0.023 v0.206 H${x1a + 0.032} v-0.206 l0.0505 -0.05 H${x2a - 0.032 - 0.0505} l0.0505 0.05 m0.032 0.236 H${x1a} M0.286 ${y1a} v0.0685 H-0.286 v-0.0685 H0.286 M-0.167 ${wheel} M0.227 ${wheel} M-0.04 ${y1 + 0.137} ${circle} M-0.04 ${y1 + 0.468} ${circle}`;
}

function titanTpl(length, width, vcenter) {
  // counter-clockwise
  const x1 = -width / 2 + 0.027;
  const x1a = x1 + 0.103;
  const x2 = -x1;
  const x2a = -x1a;

  const y1 = -(length - vcenter) + 0.03;
  const y1a = y1 + 0.064;
  const y2 = vcenter - 0.03;
  const y2a = y2 - 0.064;

  const wheel = '-0.1175 v0.235 h0.05 v-0.235 h-0.05';
  const circle = 'a0.04 0.04 0 1 0 0.08 0 a0.04 0.04 0 1 0 -0.08 0';

  return r4`M${x2} ${y1 + 0.062} c0 -0.062 -0.063 -0.062 -0.063 -0.062 H0.072 h0.015 l-0.087 -0.025 l-0.087 0.025 h0.015 H${x1a + 0.035} l-0.038 -0.03 h-0.08 c-0.047 0 -0.047 0.055 -0.047 0.055 v0.107 l0.027 0.042 V${y2 - 0.062} c0 0.062 0.063 0.062 0.063 0.062 H${x2a - 0.035} l0.038 0.03 h0.08 c0.047 0 0.047 -0.055 0.047 -0.055 v-0.107 l-0.027 -0.042 V${y1 + 0.062} M-0.072 ${y1} v0.044 h0.144 v-0.044 M${x1a - 0.025} ${wheel} M${x2a - 0.025} ${wheel} m0.025 0 V${y1a + 0.03} c0 -0.03 -0.03 -0.03 -0.03 -0.03 H${x1a + 0.03} c-0.03 0 -0.03 0.03 -0.03 0.03 V-0.1175 m0 0.235 V${y2a - 0.03} c0 0.03 0.03 0.03 0.03 0.03 H${x2a - 0.03} c0.03 0 0.03 -0.03 0.03 -0.03 V0.1175 M${x1 + 0.0025} ${y1 + 0.04} ${circle} M${x2 - 0.0825} ${y2 - 0.04} ${circle} M-0.03 0 H0.03 M0 0.1 V-0.1 m-0.05 0 l0.1 0 l-0.05 -0.1 l-0.05 0.1`;
}

function omniTpl(length, width, vcenter) {
  // clockwise
  const x1 = -width / 2;
  const x1a = x1 + 0.0601;
  const x2 = -x1;
  const x2a = -x1a;

  const y1 = -(length - vcenter);
  const y1a = y1 + 0.0487;
  const y2 = vcenter;
  const y2a = y2 - 0.0487;

  const wheel = 'h0.107 v0.254 h-0.107 Z';
  const circle = 'a0.04 0.04 0 1 1 0.08 0 a0.04 0.04 0 1 1 -0.08 0';

  return r4`M${x1} ${y1} H${x2} V${y2} H${x1} Z M${x1a} ${y1a} H${x2a} V${y2a} H${x1a} Z M${x1a - 0.04} ${y1a} ${circle} M${x2a - 0.04} ${y2a} ${circle} M${x1 + 0.0718} ${y1 + 0.0835} ${wheel} M${x2 - 0.1788} ${y1 + 0.0835} ${wheel} M${x1 + 0.0718} ${y2 - 0.3375} ${wheel} M${x2 - 0.1788} ${y2 - 0.3375} ${wheel} M0.01 -0.25 v-0.1186 l0.0436 0.0363 a0.01 0.01 90 0 0 0.0128 -0.0154 l-0.06 -0.05 a0.0119 0.0119 90 0 0 -0.0015 -0.0009 c-0.0005 0 -0.0008 -0.0005 -0.0013 -0.0007 a0.01 0.01 90 0 0 -0.0036 -0.0007 a0.01 0.01 90 0 0 -0.0036 0.0007 c-0.0005 0 -0.0008 0.0005 -0.0013 0.0007 a0.0119 0.0119 90 0 0 -0.0015 0.0009 l-0.06 0.05 a0.01 0.01 90 0 0 -0.0036 0.0077 a0.01 0.01 90 0 0 0.0023 0.0064 a0.01 0.01 90 0 0 0.0141 0.0013 l0.0436 -0.0363 v0.1186 a0.01 0.01 90 0 0 0.02 0 Z M0.1848 0.0132 v-0.0264 h-0.0271 a0.1585 0.1585 90 0 0 -0.1445 -0.1445 v-0.0271 h-0.0264 v0.0271 a0.1585 0.1585 90 0 0 -0.1445 0.1445 h-0.0271 v0.0264 h0.0271 a0.1585 0.1585 90 0 0 0.1445 0.1445 v0.0271 h0.0264 v-0.0271 a0.1585 0.1585 90 0 0 0.1445 -0.1445 Z M0.0132 0.1313 v-0.0521 h-0.0264 v0.0521 a0.1322 0.1322 90 0 1 -0.1181 -0.1181 h0.0521 v-0.0264 h-0.0521 a0.1322 0.1322 90 0 1 0.1181 -0.1181 v0.0521 h0.0264 v-0.0521 a0.1322 0.1322 90 0 1 0.1181 0.1181 h-0.0521 v0.0264 h0.0521 a0.1322 0.1322 90 0 1 -0.1181 0.1181 Z`;
}

const dimsZalphaExtended = [1.03, 0.55, 0.51];
const dimsZalphaStandard = [0.75, 0.55, 0.23];
const dimsZethaStandard = [1.938, 0.78, 0.652];
const dimsTitanStandard = [1.36, 0.96, 0.68];
const dimsOmniStandard = [1.066, 0.55, 0.533];

function trolleyTpl(length, width, vcenter, hcenter) {
  const x1 = hcenter - width / 2;
  const x2 = hcenter + width / 2;
  const y1 = vcenter - length / 2;
  const y2 = vcenter + length / 2;
  return r4`M${x1} ${y1} H${x2} V${y2} H${x1} V${y1} v0.05 h0.05 v-0.05 v0.025 H${x2 - 0.05} v-0.025 v0.05 h0.05 h-0.025 V${y2 - 0.05} h0.025 h-0.05 v0.05 v-0.025 H${x1 + 0.05} v0.025 v-0.05 h-0.05 h0.025 V${y1 + 0.05}`;
}

function palletTpl(length, width, vcenter, hcenter) {
  const x1 = hcenter - width / 2;
  const x2 = hcenter + width / 2;
  const y1 = vcenter - length / 2;
  // const y2 = vcenter + length / 2;

  const n = Math.floor((length - 0.12) / 0.18);
  const sp = (length - 0.12 * (n + 1)) / n;

  const hbar = r4`H${x2} v0.12 H${x1} v-0.12`;
  const vbar = r4`v0.12 h0.12 v${sp} h-0.12 v${-sp} H${hcenter + 0.07} v${sp} h-0.14 v${-sp} H${x2} v${sp} h-0.12 v${-sp} H${x1} v${sp}`;

  var s = r4`M${x1} ${y1} ${hbar}`;
  for (var i = 0; i < n; i++) {
    s += ` ${vbar} ${hbar}`;
  }
  return s;
}

function round3(v) {
  return Math.round(v * 1000) / 1000;
}

function round4(v) {
  return Math.round(v * 10000) / 10000;
}

function r4(strings) {
  var result = [strings[0]];
  for (var i = 1; i < arguments.length; i++) {
    var v = arguments[i];
    if (Number.isFinite(v)) {
      v = round4(v);
    }
    result.push(v, strings[i]);
  }
  return result.join('');
}
