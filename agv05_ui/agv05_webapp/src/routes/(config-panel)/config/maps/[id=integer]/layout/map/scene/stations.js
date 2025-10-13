/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Station from '../models/station';

export default function (viz, scene) {
  /* Stations */
  var stations = scene.append('g').attr('class', 'stations');

  function modelsUpdated() {
    var data = viz.models.rawStations();
    var dataJ = viz.models.rawJunctions();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();
    var st = stations.selectAll('use').data(data).join('use').attr('class', 'station');
    st.attrs({
      transform: function (d) {
        if (d.j >= 0 && d.j < dataJ.length) {
          return `translate(${dataJ[d.j].x},${dataJ[d.j].y})rotate(${d.direction * 90})`;
        } else {
          console.log("Station's junction index out of bounds.");
          return '';
        }
      },
      'xlink:href': function (d) {
        return d.direction === Station.Direction.NA ? '#station-directionless' : '#station';
      }
    })
      .classed('active', function (d) {
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      })
      .classed('active-overlap', function (d) {
        return (
          discardObjects.every((ob) => ob.j !== d.j) &&
          (activeObjects.some((ob) => ob.j === d.j) || hoverObjects.some((ob) => ob.j === d.j))
        );
      });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
