/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

import ForbiddenZone from 'mapx-layout-editor/models/forbidden-zone';
import Junction from 'mapx-layout-editor/models/junction';
import Landmark from 'mapx-layout-editor/models/landmark';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import Path from 'mapx-layout-editor/models/path';
import Station from 'mapx-layout-editor/models/station';
import Layer from 'mapx-layout-editor/models/layer';
import TextAnnotation from 'map-annotation-editor/models/text-annotation';
import PolygonAnnotation from 'map-annotation-editor/models/polygon-annotation';
import IconAnnotation from 'map-annotation-editor/models/icon-annotation';

export default function (viz) {
  /* Model for map data */
  var junctions = [];
  var paths = [];
  var stations = [];
  var landmarks = [];
  var forbiddenZones = [];
  var noRotateZones = [];
  var activeLayer = Layer.LAYER_0;
  var visibleLayers = {};
  var textAnnotations = [];
  var polygonAnnotations = [];
  var iconAnnotations = [];
  Layer.list.forEach(([k, v]) => (visibleLayers[v] = true));

  /* Deserialize from json */
  function load(layout) {
    if (!layout) {
      return;
    }

    if (layout.junctions && Array.isArray(layout.junctions)) {
      junctions = _.map(layout.junctions, (j) => new Junction(j));
    }
    if (layout.paths && Array.isArray(layout.paths)) {
      paths = _.map(layout.paths, (p) => new Path(p));
    }
    if (layout.stations && Array.isArray(layout.stations)) {
      stations = _.map(layout.stations, (s) => new Station(s));
    }
    if (layout.landmarks && Array.isArray(layout.landmarks)) {
      landmarks = _.map(layout.landmarks, (l) => new Landmark(l));
    }
    if (layout.forbidden_zones && Array.isArray(layout.forbidden_zones)) {
      forbiddenZones = _.map(layout.forbidden_zones, (f) => new ForbiddenZone(f));
    }
    if (layout.no_rotate_zones && Array.isArray(layout.no_rotate_zones)) {
      noRotateZones = _.map(layout.no_rotate_zones, (n) => new NoRotateZone(n));
    }
    if (layout.annotations) {
      let annotations = layout.annotations;
      if (annotations.textAnnotations && Array.isArray(annotations.textAnnotations)) {
        textAnnotations = _.map(annotations.textAnnotations, (t) => new TextAnnotation(t));
      }
      if (annotations.polygonAnnotations && Array.isArray(annotations.polygonAnnotations)) {
        polygonAnnotations = _.map(
          annotations.polygonAnnotations,
          (p) => new PolygonAnnotation(p)
        );
      }
      if (annotations.iconAnnotations && Array.isArray(annotations.iconAnnotations)) {
        iconAnnotations = _.map(annotations.iconAnnotations, (i) => new IconAnnotation(i));
      }
    }

    viz.modelsUpdated();
    window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  /* Helper functions */
  function computeBoundingBox() {
    if (
      !junctions.length &&
      !landmarks.length &&
      !forbiddenZones.length &&
      !noRotateZones.length
    ) {
      return null;
    }
    var bbox = {
      top: -10000,
      bottom: 10000,
      left: -10000,
      right: 10000
    };
    for (let junction of junctions) {
      if (junction.x > bbox.top) {
        bbox.top = junction.x;
      }
      if (junction.x < bbox.bottom) {
        bbox.bottom = junction.x;
      }
      if (junction.y > bbox.left) {
        bbox.left = junction.y;
      }
      if (junction.y < bbox.right) {
        bbox.right = junction.y;
      }
    }
    for (let landmark of landmarks) {
      if (landmark.x > bbox.top) {
        bbox.top = landmark.x;
      }
      if (landmark.x < bbox.bottom) {
        bbox.bottom = landmark.x;
      }
      if (landmark.y > bbox.left) {
        bbox.left = landmark.y;
      }
      if (landmark.y < bbox.right) {
        bbox.right = landmark.y;
      }
    }
    for (let zone of forbiddenZones) {
      for (let pt of zone.points) {
        if (pt[0] > bbox.top) {
          bbox.top = pt[0];
        }
        if (pt[0] < bbox.bottom) {
          bbox.bottom = pt[0];
        }
        if (pt[1] > bbox.left) {
          bbox.left = pt[1];
        }
        if (pt[1] < bbox.right) {
          bbox.right = pt[1];
        }
      }
    }
    for (let zone of noRotateZones) {
      for (let pt of zone.points) {
        if (pt[0] > bbox.top) {
          bbox.top = pt[0];
        }
        if (pt[0] < bbox.bottom) {
          bbox.bottom = pt[0];
        }
        if (pt[1] > bbox.left) {
          bbox.left = pt[1];
        }
        if (pt[1] < bbox.right) {
          bbox.right = pt[1];
        }
      }
    }
    return bbox;
  }

  viz.models = {
    load: load,
    computeBoundingBox: computeBoundingBox,

    rawJunctions: () => junctions,
    rawPaths: () => paths,
    rawStations: () => stations,
    rawLandmarks: () => landmarks,
    rawForbiddenZones: () => forbiddenZones,
    rawNoRotateZones: () => noRotateZones,
    rawActiveLayer: () => activeLayer,
    rawVisibleLayers: () => visibleLayers,

    rawTextAnnotations: () => textAnnotations,
    rawPolygonAnnotations: () => polygonAnnotations,
    rawIconAnnotations: () => iconAnnotations
  };
}
