/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

import { getEnv } from 'stores/auth';
import Action from 'task-template-editor/models/action';

export default class Teleport {
  constructor(options) {
    options = options || {};
    this.start = options.start;
    this.end = options.end;
    this.preAction = new Action(options.preAction);
    this.action = new Action(options.action);
    this.alignStationType =
      options.alignStationType === undefined
        ? getEnv('TRACKLESS')
          ? -1
          : 0
        : options.alignStationType;
    this.nonStopTransition = !!options.nonStopTransition;
    this.autoResetAgvPosition =
      options.autoResetAgvPosition === undefined ? 1 : options.autoResetAgvPosition;
    this.validateRfid = options.validateRfid === undefined || !!options.validateRfid;
    this.distance = options.distance || 100.0;
  }

  __pick__() {
    var __attrs__ = [
      'start',
      'end',
      'preAction',
      'action',
      'alignStationType',
      'nonStopTransition',
      'autoResetAgvPosition',
      'validateRfid',
      'distance'
    ];
    return Object.assign(_.pick(this, __attrs__), {
      preAction: this.preAction.__pick__(),
      action: this.action.__pick__()
    });
  }
}
