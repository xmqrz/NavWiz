/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class BarAR extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <rect x="70" y="10" width="60" height="60" style="fill:rgb(0,0,0); stroke:rgb(0,0,0)" />
        <rect x="80" y="20" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="80" y="50" width="40" height="10" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <rect x="100" y="30" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" /> 
        <rect x="110" y="20" width="10" height="20" style="fill:rgb(255,255,255); stroke:rgb(255,255,255)" />
        <line x1="0" y1="40" x2="230" y2="40" style="stroke:rgb(255,0,0);"/>
        <line x1="80" y1="40" x2="120" y2="40" style="stroke:rgb(0,255,0);"/>
        <line x1="100" y1="20" x2="100" y2="40" style="stroke:rgb(0,255,0);"/> 
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
    this.additionalSensorOptions = [
      {
        value: '',
        display: '-'
      },
      {
        value: 'laser1',
        display: 'Laser 1'
      },
      {
        value: 'laser2',
        display: 'Laser 2'
      },
      {
        value: 'laser3',
        display: 'Laser 3'
      },
      {
        value: 'laser4',
        display: 'Laser 4'
      },
      {
        value: 'laser5',
        display: 'Laser 5'
      },
      {
        value: 'laser6',
        display: 'Laser 6'
      }
    ];
    this.allowAngleConfiguration = false;
    this.allowIdConfiguration = true;
    this.singleMarkerOnly = true;
    this.additonalSensor = true;
  }
}

export default BarAR;
