/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import PolygonAnnotation from '../models/polygon-annotation';
import contextMenuF from '$lib/shared/context-menu.js';
let contextMenu = contextMenuF();

var deleteMenuHtml = `
<li><button type="button" class="remove">Remove</button></li>
`;

export default function (viz, scene) {
  /* Polygon Annotations */
  var polygonAnnotations = scene.append('g').attr('class', 'polygon-annotations');

  var zones = polygonAnnotations.append('g').attr('class', 'zones');
  var points = polygonAnnotations.append('g').attr('class', 'zone-active');
  var construct = polygonAnnotations.append('circle').attrs({
    class: 'construct',
    display: 'none'
  });

  var activeZone;
  var activeCoord = -1;
  var constructPoints = [];
  var constructIdx = -1;
  var dragAllStartPt;

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered() {
    /* jshint validthis: true */
    var pt;
    if (activeZone) {
      pt = viz.zoom.pixelToCoord(this.mouse);
      [pt, constructIdx] = viz.models.hitTestFzPerimeter(activeZone, pt);
    }
    drawConstruct(pt);
    pt = !!pt;
    polygonAnnotations.classed('construct-active', pt);
    viz.style('cursor', 'crosshair');
  }

  function hoverended() {
    drawConstruct();
  }

  /* Drag event */
  var drag = d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended);
  construct.call(drag);

  function dragstarted(event, d) {
    /* jshint validthis: true */
    let idx = constructPoints.indexOf(d);
    event.sourceEvent.stopPropagation();
    drawConstruct();
    if (this === construct.node()) {
      constructPoints.splice(constructIdx, 0, d);
      setActiveCoord(constructIdx);
    } else {
      setActiveCoord(idx);
    }
    this.initial = d.slice();
    polygonAnnotations.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragged(event) {
    /* jshint validthis: true */
    var pt = viz.zoom.computeSnap([event.x, event.y]);
    var oldPt = constructPoints[activeCoord];
    constructPoints[activeCoord] = pt;

    if (_limitSimplePolygon(constructPoints)) {
      constructPoints.valid = pt[0] !== this.initial[0] || pt[1] !== this.initial[1];
      viz.style('cursor', 'move');
      modelsUpdated();
    } else {
      constructPoints.valid = false;
      constructPoints[activeCoord] = oldPt;
      viz.style('cursor', 'not-allowed');
    }
  }

  function dragended() {
    if (constructPoints.valid) {
      viz.models.push('dragPolygonAnnotationCorner');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      viz.models.updatePolygonAnnotation(activeZone);
    } else {
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    polygonAnnotations.classed('drag-active', false);
    enableEvents();
  }

  function _limitSimplePolygon(points) {
    for (var i = points.length - 1, j = 0; j < points.length - 2; i = j++) {
      for (var k = j + 1, l = j + 2; l < points.length; k = l++) {
        if (l === i) {
          continue;
        }
        if (viz.models.lineSegmentsIntersect(points[i], points[j], points[k], points[l])) {
          return false;
        }
      }
    }
    return true;
  }

  var dragAll = d3
    .drag()
    .on('start', dragAllStarted)
    .on('drag', dragAllDragged)
    .on('end', dragAllEnded);

  function dragAllStarted(event, d) {
    event.sourceEvent.stopPropagation();
    dragAllStartPt = [event.x, event.y];
    setActiveZone(d);
    setActiveCoord(-1);

    polygonAnnotations.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragAllDragged(event) {
    var pt = viz.zoom.computeSnap([event.x - dragAllStartPt[0], event.y - dragAllStartPt[1]]);
    for (var i = 0; i < constructPoints.length; ++i) {
      constructPoints[i] = [activeZone.points[i][0] + pt[0], activeZone.points[i][1] + pt[1]];
    }
    constructPoints.valid = pt[0] !== 0 || pt[1] !== 0;
    modelsUpdated();
  }

  function dragAllEnded() {
    if (constructPoints.valid) {
      viz.models.push('dragPolygonAnnotation');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      viz.models.updatePolygonAnnotation(activeZone);
    } else {
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    polygonAnnotations.classed('drag-active', false);
    enableEvents();
  }

  /* Context menu */
  var deleteMenu = $(deleteMenuHtml);
  deleteMenu.find('.remove').on('click', function () {
    var idx = deleteMenu.data('i');
    if (
      Number.isInteger(idx) &&
      activeZone &&
      idx >= 0 &&
      idx < constructPoints.length &&
      constructPoints.length > 3
    ) {
      constructPoints.splice(idx, 1);
      if (_limitSimplePolygon(constructPoints)) {
        viz.models.push('removePolygonAnnotationCorner');
        activeZone.points = constructPoints.slice();
        viz.models.updatePolygonAnnotation(activeZone);
      } else {
        constructPoints = activeZone.points.slice();
      }
    }
    contextMenu.hide();
    return false;
  });
  contextMenu.setContents(deleteMenu);

  function showDeleteMenu(event, d) {
    let idx = constructPoints.indexOf(d);
    deleteMenu.data('i', idx);
    contextMenu.show(event.pageX, event.pageY);
  }

  function drawConstruct(pt) {
    if (!pt) {
      construct.attr('display', 'none');
      return;
    }
    construct.datum(pt).attrs({
      cx: (d) => d[0],
      cy: (d) => d[1],
      display: 'block'
    });
  }

  function modelsUpdated() {
    var data = viz.models.rawPolygonAnnotations();
    var activeObject = scene.activeObject.get();
    if (activeObject instanceof PolygonAnnotation) {
      if (activeZone !== activeObject) {
        activeZone = activeObject;
        constructPoints = activeZone.points.slice();
      }
    } else {
      activeZone = null;
      constructPoints = [];
    }

    var z = zones.selectAll('polygon').data(data).join('polygon').call(dragAll);
    z.attrs({
      fill: (d) => d.fill,
      'fill-opacity': (d) => d.fillOpacity,
      points: (d) => (d === activeObject ? constructPoints : d.points)
    });

    var p = points
      .selectAll('circle')
      .data(constructPoints)
      .join('circle')
      .attrs({
        r: _getRadius(),
        'stroke-width': _getStrokeWidth()
      })
      .call(drag)
      .on('contextmenu', showDeleteMenu);
    p.attrs({
      cx: (d) => d[0],
      cy: (d) => d[1]
    });
    p.classed('active', (d, idx) => idx === activeCoord);
  }

  function zoomed() {
    var a = {
      r: _getRadius(),
      'stroke-width': _getStrokeWidth()
    };
    points.selectAll('circle').attrs(a);
    construct.attrs(a);
  }

  function _getRadius() {
    return 4 / viz.meta.scale;
  }

  function _getStrokeWidth() {
    return 2 / viz.meta.scale;
  }

  function enableEvents(enable) {
    if (!arguments.length || enable) {
      viz.call(hover);
      viz.style('cursor', 'default');
      viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.
      polygonAnnotations.classed('active', true);
    } else {
      polygonAnnotations.classed('active', false);
    }
  }

  function setActiveZone(ob) {
    scene.activeObject.set(ob);
  }

  function setActiveCoord(idx) {
    if (activeCoord === idx) {
      return;
    }
    activeCoord = idx;
    modelsUpdated();
    contextMenu.hide();
  }

  function toggle(state) {
    polygonAnnotations.classed('hidden', !state);
  }

  return {
    construct: function (newParent) {
      points.remove();
      construct.remove();

      points = newParent.append('g').attr('class', 'zone-active');
      construct = newParent.append('circle').attrs({
        class: 'construct',
        display: 'none'
      });
      construct.call(drag);
    },
    modelsUpdated: modelsUpdated,
    zoomed: zoomed,
    enableEvents: enableEvents,
    toggle: toggle
  };
}
