/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class Reflector extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <line x1="70" y1="40" x2="130" y2="40" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="20" x2="100" y2="40" style="stroke:rgb(0,255,0);"/>
        <circle cx="40" cy="40" r="20" fill="blue"></circle>
        <path d="M 40,50 C 40,45 45,40 50,40 45,40 40,35 40,30 40,35 35,40 30,40 35,40 40,45 40,50 Z" fill="white" stroke="none"></path>
        <circle cx="160" cy="40" r="20" fill="blue"></circle>
        <path d="M 160,50 C 160,45 165,40 170,40 165,40 160,35 160,30 160,35 155,40 150,40 155,40 160,45 160,50 Z" fill="white" stroke="none"></path>
        </svg>`;
  }
}

export default Reflector;
