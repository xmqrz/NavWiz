/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import vizF from './viz';
import modelsF from './models';

export default function (outer) {
  var editor = $(outer);
  var viz = vizF(editor);
  modelsF(viz);

  return {
    load: (options) => viz.models.load(options)
  };
}
