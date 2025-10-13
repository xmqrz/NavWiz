/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

// Marker.js module
class Marker {
  constructor() {
    this.profileOptions = [
      {
        value: 1,
        display: 'Profile 1'
      },
      {
        value: 2,
        display: 'Profile 2'
      },
      {
        value: 3,
        display: 'Profile 3'
      },
      {
        value: 4,
        display: 'Profile 4'
      },
      {
        value: 5,
        display: 'Profile 5'
      }
    ];
    this.sensorOptions = [
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
    this.additionalSensorOptions = [];
    this.svg = '';
    this.allowProfileConfiguration = true;
    this.allowAngleConfiguration = true;
    this.allowRangeMaxConfiguration = true;
    this.allowIdConfiguration = false;
    this.singleMarkerOnly = false;
    this.additonalSensor = false;
    this.minAngle = -360;
    this.maxAngle = 360;
  }
}

export default Marker;
