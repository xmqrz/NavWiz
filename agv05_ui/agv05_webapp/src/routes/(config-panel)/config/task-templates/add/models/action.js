/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

export default class Action {
  constructor(options) {
    options = options || {};
    this.skillId = options.skillId || null;
    this.params = Object.assign({}, options.params);
    this.outcomes = Object.assign({}, options.outcomes);
    this.tooltip = options.tooltip || '';
    this.pos = options.pos || [0.0, 0.0];

    // manual scene rendering cache data.
    this.renderMeta = Object.assign(
      {
        width: 300,
        height: 200,
        displayName: '',
        titleWidth: 200,
        titleHeight: 31,
        paramWidth: 200,
        paramHeight: 100,
        paramsText: [],
        outcomeWidth: 200,
        outcomeHeight: 28,
        outcomesRelativePos: [],
        outcomesWidth: [],
        outcomesText: [],
        outcomes: [],
        url: '',
        color: 'khaki'
      },
      options.renderMeta || {}
    );
  }
  __pick__() {
    var __attrs__ = ['skillId', 'params', 'outcomes', 'tooltip', 'pos'];
    return _.pick(this, __attrs__);
  }
}
