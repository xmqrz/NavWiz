/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

import Param from './param.js';

export default class TaskTemplateMeta {
  constructor(options) {
    options = options || {};

    this.id = Number.isInteger(options.id) ? '_ttpk_' + options.id : options.id;
    this.raw_id = options.id;
    this.name = options.name;
    if (options.metadata) {
      var metadata = JSON.parse(options.metadata);
      options.params = metadata.params;
      options.outcomes = metadata.outcomes;
      options.create_suspended = metadata.create_suspended;
    }
    this.params = _.map(options.params || [], (p) => new Param(p));
    this.outcomes = options.outcomes || [];
    this.is_top_level = options.is_top_level;
    this.is_active = options.is_active;
    this.create_suspended = options.create_suspended;
    this.category = options.category || '';
  }
}
