/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import Marker from './marker.js';

class Pallet extends Marker {
  constructor() {
    super();
    this.svg = `<svg width="200" height="80">
        <path style="stroke:#bc9554" d="m50 39.5 55-22m-55 22 45 23m0 0 55-20m-45-25 45 25m-100-3v15m45 8V74m55-31.5v11M74 66v-7.5m0 7.5 9-3m-9 3-5-3m0 0v-6m-19-2.5 6 3m.5 0 7.5-2m-8 3V51m0 0 13 6.5M95 74l-7.5-3.5m0 0v-5m0 0L74 59m21 15 7.5-3m0 0v-5m0 0 18.5-6.5m0 0v5m0 0-6-2.5m6 2.5 6-2m0 0v-4m0 0 17.5-7m0 0v5m0-.5-7-2m7 2 6-2.5M50 39.5l55-22m-46 27 54.5-22M68 49.5l54-22m-45 27 53.5-22M86.5 59l53-21.5"/>
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
    this.minAngle = -90;
    this.maxAngle = 90;
  }
}

export default Pallet;
