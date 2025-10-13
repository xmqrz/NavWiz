/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';
import Param from '$lib/shared/models/param.js';

export default class SkillDesc {
  constructor(options) {
    options = options || {};
    this.id = options.id;
    this.name = options.name;
    this.params = _.map(options.params || [], (paramJson) => new Param(JSON.parse(paramJson)));
    this.outcomes = options.outcomes || [];
    this.mutexes = options.mutexes || [];
  }
}
