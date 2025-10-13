/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

import Junction from 'map-layout-editor/models/junction';
import Path from 'map-layout-editor/models/path';
import Station from 'map-layout-editor/models/station';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import TextAnnotation from './text-annotation';
import PolygonAnnotation from './polygon-annotation';
import IconAnnotation from './icon-annotation';
import hitTest from './hit-test';
import hitTestMap from 'map-layout-editor/models/hit-test';

const MAX_UNDO_LEVELS = 150;

export default function (viz) {
  /* Model for map data */
  var $viz = $(viz.node());
  var junctions = [];
  var paths = [];
  var stations = [];
  var noRotateZones = [];
  var textAnnotations = [];
  var polygonAnnotations = [];
  var iconAnnotations = [];
  var undoBuffer = [];
  var redoBuffer = [];
  var dirty = false;

  /* Deserialize from database */
  function load(options) {
    if (options.structure) {
      if (options.structure.junctions && Array.isArray(options.structure.junctions)) {
        junctions = _.map(options.structure.junctions, (j) => new Junction(j));
      }
      if (options.structure.paths && Array.isArray(options.structure.paths)) {
        paths = _.map(options.structure.paths, (p) => new Path(p));
      }
      if (options.structure.stations && Array.isArray(options.structure.stations)) {
        stations = _.map(options.structure.stations, (s) => new Station(s));
      }
      if (
        options.structure.no_rotate_zones &&
        Array.isArray(options.structure.no_rotate_zones)
      ) {
        noRotateZones = _.filter(
          _.map(options.structure.no_rotate_zones, (n) => new NoRotateZone(n)),
          (zone) => zone.points && zone.points.length && zone.points.length >= 3
        );
      }
    }
    if (options.annotations) {
      textAnnotations = _.map(
        options.annotations.textAnnotations,
        (t) => new TextAnnotation(t)
      );
      polygonAnnotations = _.map(
        options.annotations.polygonAnnotations,
        (p) => new PolygonAnnotation(p)
      );
      iconAnnotations = _.map(
        options.annotations.iconAnnotations,
        (i) => new IconAnnotation(i)
      );
    }
    viz.modelsLoaded();
    window.requestAnimationFrame(viz.zoom.zoomFit);
  }

  /* Serialize to database */
  function getAnnotation() {
    return {
      textAnnotations: _.map(textAnnotations, (textAnnotation) =>
        _.pick(textAnnotation, TextAnnotation.__attrs__)
      ),
      polygonAnnotations: _.map(polygonAnnotations, (polygonAnnotation) =>
        _.pick(polygonAnnotation, PolygonAnnotation.__attrs__)
      ),
      iconAnnotations: _.map(iconAnnotations, (iconAnnotation) =>
        _.pick(iconAnnotation, IconAnnotation.__attrs__)
      )
    };
  }

  /* Manipulation: TextAnnotation */
  function addTextAnnotation(textAnnotation, pushHistory = true) {
    if (pushHistory) {
      push('addTextAnnotation');
    }
    var idx = textAnnotations.indexOf(textAnnotation);
    if (idx >= 0) {
      return idx;
    }
    textAnnotations.push(new TextAnnotation(textAnnotation));
    triggerDirty();
    return textAnnotations.length - 1;
  }

  function updateTextAnnotation(textAnnotation) {
    var idx = textAnnotations.indexOf(textAnnotation);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeTextAnnotation(textAnnotation) {
    var idx;
    if (Number.isInteger(textAnnotation)) {
      idx = textAnnotation;
    } else {
      idx = textAnnotations.indexOf(textAnnotation);
    }
    if (idx < 0 || idx >= textAnnotations.length) {
      return;
    }
    textAnnotations.splice(idx, 1);
    triggerDirty();
  }

  /* Manipulation: Polygon Annotation */
  function addPolygonAnnotation(polygonAnnotation) {
    push('addPolygonAnnotation');
    var idx = polygonAnnotations.indexOf(polygonAnnotation);
    if (idx >= 0) {
      return idx;
    }
    polygonAnnotations.push(new PolygonAnnotation(polygonAnnotation));
    triggerDirty();
    return polygonAnnotations.length - 1;
  }

  function updatePolygonAnnotation(polygonAnnotation) {
    var idx = polygonAnnotations.indexOf(polygonAnnotation);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removePolygonAnnotation(polygonAnnotation) {
    var idx;
    if (Number.isInteger(polygonAnnotation)) {
      idx = polygonAnnotation;
    } else {
      idx = polygonAnnotations.indexOf(polygonAnnotation);
    }
    if (idx < 0 || idx >= polygonAnnotations.length) {
      return;
    }
    polygonAnnotations.splice(idx, 1);
    triggerDirty();
  }

  /* Manipulation: Icon */
  function addIconAnnotation(iconAnnotation, pushHistory = true) {
    if (pushHistory) {
      push('addIconAnnotation');
    }
    var idx = iconAnnotations.indexOf(iconAnnotation);
    if (idx >= 0) {
      return idx;
    }
    iconAnnotations.push(new IconAnnotation(iconAnnotation));
    triggerDirty();
    return iconAnnotations.length - 1;
  }

  function updateIconAnnotation(iconAnnotation) {
    var idx = iconAnnotations.indexOf(iconAnnotation);
    if (idx >= 0) {
      triggerDirty();
      return idx;
    }
    return false;
  }

  function removeIconAnnotation(iconAnnotation) {
    var idx;
    if (Number.isInteger(iconAnnotation)) {
      idx = iconAnnotation;
    } else {
      idx = iconAnnotations.indexOf(iconAnnotation);
    }
    if (idx < 0 || idx >= iconAnnotations.length) {
      return;
    }
    iconAnnotations.splice(idx, 1);
    triggerDirty();
  }

  /* Manupulation: Undo & Redo */
  var canUndo = () => !!undoBuffer.length;
  var canRedo = () => !!redoBuffer.length;

  function undo() {
    if (!canUndo()) {
      return;
    }
    _preserveStructure(redoBuffer, 'undo');
    _restoreStructure(undoBuffer);
    $viz.trigger('models.undoBuffer');
  }

  function redo() {
    if (!canRedo()) {
      return;
    }
    _preserveStructure(undoBuffer, 'redo');
    _restoreStructure(redoBuffer);
    $viz.trigger('models.undoBuffer');
  }

  function push(name) {
    _preserveStructure(undoBuffer, name);
    redoBuffer = [];
    $viz.trigger('models.undoBuffer');
  }

  function _preserveStructure(buffer, name) {
    buffer.push({
      name: name,
      textAnnotations: JSON.stringify(textAnnotations),
      polygonAnnotations: JSON.stringify(polygonAnnotations),
      iconAnnotations: JSON.stringify(iconAnnotations)
    });
    if (buffer.length > MAX_UNDO_LEVELS) {
      buffer.shift();
    }
  }

  function _restoreStructure(buffer) {
    var op = buffer.pop();
    textAnnotations = _.map(JSON.parse(op.textAnnotations), (t) => new TextAnnotation(t));
    polygonAnnotations = _.map(
      JSON.parse(op.polygonAnnotations),
      (p) => new PolygonAnnotation(p)
    );
    iconAnnotations = _.map(JSON.parse(op.iconAnnotations), (i) => new IconAnnotation(i));
    viz.scene.activeObject.set(null);
    triggerDirty();
  }

  function clearAll() {
    if (textAnnotations.length || polygonAnnotations.length || iconAnnotations.length) {
      push('clearAll');
      textAnnotations = [];
      polygonAnnotations = [];
      iconAnnotations = [];
      viz.scene.activeObject.set(null);
      triggerDirty();
    }
  }

  function triggerDirty() {
    externalTriggerDirty();
    viz.modelsUpdated();
  }

  function externalTriggerDirty() {
    if (!dirty) {
      dirty = true;
      $viz.trigger('models.dirty');
    }
  }

  /* Helper functions */
  function computeBoundingBox() {
    if (!junctions.length && !noRotateZones.length) {
      return null;
    }
    var bbox = {
      top: 10000,
      bottom: -10000,
      left: 10000,
      right: -10000
    };
    for (let junction of junctions) {
      if (junction.x < bbox.left) {
        bbox.left = junction.x;
      }
      if (junction.x > bbox.right) {
        bbox.right = junction.x;
      }
      if (junction.y < bbox.top) {
        bbox.top = junction.y;
      }
      if (junction.y > bbox.bottom) {
        bbox.bottom = junction.y;
      }
    }
    for (let zone of noRotateZones) {
      for (let pt of zone.points) {
        if (pt[0] < bbox.left) {
          bbox.left = pt[0];
        }
        if (pt[0] > bbox.right) {
          bbox.right = pt[0];
        }
        if (pt[1] < bbox.top) {
          bbox.top = pt[1];
        }
        if (pt[1] > bbox.bottom) {
          bbox.bottom = pt[1];
        }
      }
    }
    return bbox;
  }

  viz.models = {
    load: load,
    getAnnotation: getAnnotation,

    hitTestTextAnnotation: function (testTA) {
      return hitTest.textAnnotation(textAnnotations, testTA);
    },
    hitTestIconAnnotation: function (testIA) {
      return hitTest.iconAnnotation(iconAnnotations, testIA);
    },
    hitTestRect: function (testPt1, testPt2) {
      return hitTest.rect(
        textAnnotations,
        iconAnnotations,
        testPt1,
        testPt2,
        viz.zoom.getSnapDistance() / 2
      );
    },
    hitTestFzPerimeter: function (fz, testZ) {
      return hitTestMap.regionPerimeter(fz.points, testZ, viz.zoom.getSnapDistance() / 2);
    },
    lineSegmentsIntersect: hitTestMap.lineSegmentsIntersect,

    addTextAnnotation: addTextAnnotation,
    updateTextAnnotation: updateTextAnnotation,
    removeTextAnnotation: removeTextAnnotation,
    addPolygonAnnotation: addPolygonAnnotation,
    updatePolygonAnnotation: updatePolygonAnnotation,
    removePolygonAnnotation: removePolygonAnnotation,
    addIconAnnotation: addIconAnnotation,
    updateIconAnnotation: updateIconAnnotation,
    removeIconAnnotation: removeIconAnnotation,

    canUndo: canUndo,
    canRedo: canRedo,
    undo: undo,
    redo: redo,
    push: push,

    clearAll: clearAll,
    triggerDirty: triggerDirty,
    externalTriggerDirty: externalTriggerDirty,

    computeBoundingBox: computeBoundingBox,

    rawJunctions: () => junctions,
    rawPaths: () => paths,
    rawStations: () => stations,
    rawNoRotateZones: () => noRotateZones,
    rawTextAnnotations: () => textAnnotations,
    rawPolygonAnnotations: () => polygonAnnotations,
    rawIconAnnotations: () => iconAnnotations
  };
}
