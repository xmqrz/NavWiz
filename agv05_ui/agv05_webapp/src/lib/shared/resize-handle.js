/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

export default function (viz) {
  var height;
  var $viz = $(viz);
  var $handle = $('<div class="resize-handle">');
  $handle.insertAfter($viz);

  var handle = d3.select($handle[0]);

  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);
  handle.call(track);

  function trackstarted() {
    height = $viz.height();
  }

  function tracked() {
    var yd = this.a1[1] - this.a0[1];
    var newHeight = height + yd;
    $viz.height(newHeight);
  }

  function trackended() {
    $viz.trigger('resize');
  }
}
