/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, hud) {
  /* Mini Axis */
  var miniAxis = hud.append('g').attrs({
    class: 'mini-axis',
    transform: 'translate(18,-10)'
  });
  var _hitAssist = miniAxis.append('rect').attrs({
    x: 0,
    y: -20,
    width: 20,
    height: 20
  });
  var _sprite = miniAxis.append('use').attr('xlink:href', '#mini-axis');

  miniAxis.on('click', function (event) {
    // Todo: rotate axis 180 degrees.
    event.stopPropagation();
  });

  return miniAxis;
}
