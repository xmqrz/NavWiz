/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class BarReflector extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <circle cx="100" cy="40" r="20" fill="blue"></circle>
        <path d="M 100,50 C 100,45 105,40 110,40 105,40 100,35 100,30 100,35 95,40 90,40 95,40 100,45 100,50 Z" fill="white" stroke="none"></path>
        <line x1="0" y1="40" x2="230" y2="40" style="stroke:rgb(255,0,0);"/>
        <line x1="70" y1="40" x2="130" y2="40" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="20" x2="100" y2="40" style="stroke:rgb(0,255,0);"/>
        </svg>`;
  }
}

export default BarReflector;
