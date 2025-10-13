/*
 * Copyright (c) 2021, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

import Action from '../../../../routes/(config-panel)/config/task-templates/add/models/action';
import WebServerTaskTemplateMeta from '$lib/shared/models/task-template-meta.js';

export default class TaskTemplateMeta extends WebServerTaskTemplateMeta {
  constructor(options) {
    super(options);
    if (!(this instanceof TaskTemplateMeta)) {
      return new TaskTemplateMeta(options);
    }
    options = options || {};
    let first = true;
    this.actions = _.map(options.actions, function (action, i) {
      if (first) {
        first = false;
        if (Number.isInteger(action)) {
          return action;
        } else {
          return null;
        }
      }
      return new Action(action);
    });
  }
}
