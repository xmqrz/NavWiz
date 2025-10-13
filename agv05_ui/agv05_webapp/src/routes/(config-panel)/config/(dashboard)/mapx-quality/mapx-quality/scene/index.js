/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import qmapF from './qmap';

export default function (viz) {
  var scene = viz.scene;
  var qmap = qmapF(viz, scene);

  scene.updateQmap = qmap.updateQmap;
  scene.clearQmap = qmap.clearQmap;

  return scene;
}
