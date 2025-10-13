/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import * as _ from 'lodash-es';

import Action from 'task-template-editor/models/action';

export default class TransitionTrigger {
  constructor(options) {
    options = options || {};
    this.start = options.start;
    this.end = options.end;
    this.startAction = new Action(options.startAction);
    this.endAction = new Action(options.endAction);
    this.cancelNonStopTransition =
      options.cancelNonStopTransition === undefined ? true : !!options.cancelNonStopTransition;
    this.applicableMotion =
      options.applicableMotion === undefined
        ? [0, 1, 2, 3, 4]
        : options.applicableMotion.map((item) => Number.parseInt(item, 10));
  }

  __pick__() {
    var __attrs__ = [
      'start',
      'end',
      'startAction',
      'endAction',
      'cancelNonStopTransition',
      'applicableMotion'
    ];
    return Object.assign(_.pick(this, __attrs__), {
      startAction: this.startAction.__pick__(),
      endAction: this.endAction.__pick__()
    });
  }
}
