/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';
import Station from '../models/station';

export default function (viz, scene, muted = false) {
  /* Stations */
  var stations = scene.append('g').attr('class', !muted ? 'stations' : 'stations-muted');

  function modelsUpdated() {
    var visibleLayers = viz.models.rawVisibleLayers();
    var data = viz.models.rawStations();
    data = _.reduce(
      data,
      function (result, d, idx) {
        if (visibleLayers[d.layer] !== muted) {
          result.push([d, idx]);
        }
        return result;
      },
      []
    );
    var dataJ = viz.models.rawJunctions();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();
    var aoAligned = scene.activeObject.getAligned();
    var hoAligned = scene.hoverObject.getAligned();
    var st = stations.selectAll('use').data(data).join('use');
    st.attrs({
      class: ([d, _]) => `station layer-${d.layer}`,
      transform: function ([d, _]) {
        if (d.j >= 0 && d.j < dataJ.length) {
          return `translate(${dataJ[d.j].x},${dataJ[d.j].y})rotate(${d.heading})`;
        } else {
          console.log("Station's junction index out of bounds.");
          return '';
        }
      },
      'xlink:href': function ([d, _]) {
        return d.heading === Station.Heading.NA ? '#station-headless' : '#station';
      }
    })
      .classed('muted', muted)
      .classed('active', function ([d, _]) {
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      })
      .classed('active-overlap', function ([d, _]) {
        return (
          discardObjects.every((ob) => ob.j !== d.j) &&
          (activeObjects.some((ob) => ob.j === d.j) || hoverObjects.some((ob) => ob.j === d.j))
        );
      })
      .classed('aligned', function ([d, idx]) {
        return (
          aoAligned.stations[idx] ||
          hoAligned.stations[idx] ||
          (aoAligned.ob && d === scene.activeObject.get()) ||
          (hoAligned.ob && d === scene.hoverObject.get())
        );
      });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
