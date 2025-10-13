/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class AR extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <rect x="10" y="10" width="60" height="60" style="fill:rgb(0,0,0); stroke:rgb(0,0,0)" />
        <rect x="20" y="20" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="20" y="50" width="40" height="10" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="40" y="30" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" /> 
        <rect x="50" y="20" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <line x1="80" y1="40" x2="120" y2="40" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="20" x2="100" y2="40" style="stroke:rgb(0,255,0);"/>
        <rect x="130" y="10" width="60" height="60" style="fill:rgb(0,0,0); stroke:rgb(0,0,0)" />
        <rect x="140" y="20" width="10" height="30" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="140" y="50" width="20" height="10" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="160" y="30" width="10" height="10" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" /> 
        <rect x="170" y="20" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />  
        </svg>`;
    this.sensorOptions = [
      {
        value: 'camera1',
        display: 'Camera 1'
      },
      {
        value: 'camera2',
        display: 'Camera 2'
      },
      {
        value: 'camera3',
        display: 'Camera 3'
      },
      {
        value: 'camera4',
        display: 'Camera 4'
      },
      {
        value: 'camera5',
        display: 'Camera 5'
      }
    ];
    this.allowProfileConfiguration = true;
    this.allowAngleConfiguration = false;
    this.allowRangeMaxConfiguration = true;
    this.allowIdConfiguration = true;
  }
}

export default AR;
