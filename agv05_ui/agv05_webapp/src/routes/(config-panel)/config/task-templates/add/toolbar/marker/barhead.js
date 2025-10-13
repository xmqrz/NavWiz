/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class Barhead extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <line x1="30" y1="30" x2="30" y2="70" style="stroke:rgb(255,0,0);"/>
        <line x1="170" y1="30" x2="170" y2="70" style="stroke:rgb(255,0,0);"/>
        <line x1="70" y1="30" x2="130" y2="30" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="15" x2="100" y2="30" style="stroke:rgb(0,255,0);"/>
        </svg>`;
  }
}

export default Barhead;
