/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class VLMarker extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <line x1="20" y1="35" x2="40" y2="35" style="stroke:rgb(255,0,0);"/>
        <line x1="40" y1="35" x2="60" y2="50" style="stroke:rgb(255,0,0);"/>
        <line x1="60" y1="50" x2="80" y2="35" style="stroke:rgb(255,0,0);"/>
        <line x1="40" y1="50" x2="80" y2="50" style="stroke:rgb(0,255,0);"/>
        <line x1="60" y1="35" x2="60" y2="50" style="stroke:rgb(0,255,0);"/>
        <line x1="160" y1="35" x2="180" y2="35" style="stroke:rgb(255,0,0);"/>
        <line x1="120" y1="35" x2="140" y2="50" style="stroke:rgb(255,0,0);"/>
        <line x1="140" y1="50" x2="160" y2="35" style="stroke:rgb(255,0,0);"/>
        <line x1="120" y1="50" x2="160" y2="50" style="stroke:rgb(0,255,0);"/>
        <line x1="140" y1="35" x2="140" y2="50" style="stroke:rgb(0,255,0);"/>
        </svg>`;
  }
}

export default VLMarker;
