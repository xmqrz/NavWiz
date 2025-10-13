/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class LMarker extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <line x1="100" y1="40" x2="50" y2="60" style="stroke:rgb(255,0,0);"/>
        <line x1="100" y1="40" x2="150" y2="60" style="stroke:rgb(255,0,0);"/>
        <line x1="50" y1="40" x2="150" y2="40" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="20" x2="100" y2="40" style="stroke:rgb(0,255,0);"/>
        </svg>`;
  }
}

export default LMarker;
