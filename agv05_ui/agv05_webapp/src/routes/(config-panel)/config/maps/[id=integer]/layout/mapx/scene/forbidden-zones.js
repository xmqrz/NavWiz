/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import ForbiddenZone from '../models/forbidden-zone';
import contextMenuF from '$lib/shared/context-menu.js';
let contextMenu = contextMenuF();

var deleteMenuHtml = `
<li><a href="#" class="remove">Remove</a></li>
`;

export default function (viz, scene) {
  /* Forbidden Zones */
  var forbiddenZones = scene.append('g').attr('class', 'forbidden-zones');

  var zones = forbiddenZones.append('g').attr('class', 'zones');
  var points = forbiddenZones.append('g').attr('class', 'zone-active');
  var construct = forbiddenZones.append('circle').attrs({
    class: 'construct',
    display: 'none'
  });

  var activeZone;
  var activeCoord = -1;
  var constructPoints = [];
  var constructIdx = -1;

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
    forbiddenZones.classed('drag-active', true);
    points.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragged(event) {
    /* jshint validthis: true */
    var pt = viz.zoom.computeSnap([event.x, event.y]);
    if (constructPoints.prev) {
      activeCoord = constructPoints.prevCoord;
      constructPoints = constructPoints.prev;
    }
    var oldPt = constructPoints[activeCoord];
    constructPoints[activeCoord] = pt;

    // check if merge with neighbouring corner
    var down = (activeCoord || constructPoints.length) - 1;
    var ptDown = constructPoints[down];
    var mergeDown = pt[0] === ptDown[0] && pt[1] === ptDown[1];
    var mergeUp;
    if (!mergeDown) {
      var up = activeCoord + 1 >= constructPoints.length ? 0 : activeCoord + 1;
      var ptUp = constructPoints[up];
      mergeUp = pt[0] === ptUp[0] && pt[1] === ptUp[1];
    }

    var newPoints;
    if (mergeDown || mergeUp) {
      newPoints = _removeCoord(activeCoord);
      if (newPoints) {
        newPoints.prevCoord = activeCoord;
        newPoints.prev = constructPoints;
        constructPoints = newPoints;
        if (mergeDown) {
          activeCoord = (activeCoord || constructPoints.length) - 1;
        } else {
          if (activeCoord >= constructPoints.length) {
            activeCoord = 0;
          }
        }
      }
    }

    if (newPoints || _limitSimplePolygon(constructPoints)) {
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
      viz.models.push('dragForbiddenZoneCorner');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      constructPoints.prev = undefined;
      viz.models.updateForbiddenZone(activeZone);
    } else {
      if (constructPoints.length !== activeZone.points.length) {
        // implies activeCoord === constructIdx
        activeCoord = -1;
      }
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    forbiddenZones.classed('drag-active', false);
    points.classed('drag-active', false);
    enableEvents();
  }

  function _limitSimplePolygon(points) {
    if (points.length === 3) {
      let v1 = [points[1][0] - points[0][0], points[1][1] - points[0][1]];
      let v2 = [points[2][0] - points[0][0], points[2][1] - points[0][1]];
      return v1[0] * v2[1] - v1[1] * v2[0] !== 0; // cross product = 0 if parallel
    }
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
    self.dragAllStartPt = [event.x, event.y];
    setActiveZone(d);
    setActiveCoord(-1);

    forbiddenZones.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragAllDragged(event) {
    var pt = viz.zoom.computeSnap([
      event.x - self.dragAllStartPt[0],
      event.y - self.dragAllStartPt[1]
    ]);
    for (var i = 0; i < constructPoints.length; ++i) {
      constructPoints[i] = [activeZone.points[i][0] + pt[0], activeZone.points[i][1] + pt[1]];
    }
    constructPoints.valid = pt[0] !== 0 || pt[1] !== 0;
    modelsUpdated();
  }

  function dragAllEnded() {
    if (constructPoints.valid) {
      viz.models.push('dragForbiddenZone');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      viz.models.updateForbiddenZone(activeZone);
    } else {
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    forbiddenZones.classed('drag-active', false);
    enableEvents();
  }

  /* Context menu */
  var deleteMenu = $(deleteMenuHtml);
  var remove = deleteMenu.find('.remove').on('click', function () {
    var newPoints = deleteMenu.data('newPoints');
    if (newPoints) {
      activeCoord = -1;
      constructPoints = newPoints;
      viz.models.push('removeForbiddenZoneCorner');
      activeZone.points = constructPoints.slice();
      viz.models.updateForbiddenZone(activeZone);
    }
    contextMenu.hide();
    return false;
  });
  contextMenu.setContents(deleteMenu);

  function showDeleteMenu(event, d) {
    let idx = constructPoints.indexOf(d);
    var newPoints = _removeCoord(idx);
    deleteMenu.data('newPoints', newPoints || null);
    remove.css('cursor', newPoints ? '' : 'not-allowed');
    contextMenu.show(event.pageX, event.pageY);
  }

  function _removeCoord(idx) {
    if (
      Number.isInteger(idx) &&
      activeZone &&
      idx >= 0 &&
      idx < constructPoints.length &&
      constructPoints.length > 3
    ) {
      var points = constructPoints.slice();
      points.splice(idx, 1);
      if (_limitSimplePolygon(points)) {
        return points;
      }
    }
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
    var data = viz.models.rawForbiddenZones();
    var activeObject = scene.activeObject.get();

    if (activeObject instanceof ForbiddenZone) {
      if (activeZone !== activeObject) {
        activeZone = activeObject;
        activeCoord = -1;
        constructPoints = activeZone.points.slice();
      }
    } else {
      activeZone = null;
      constructPoints = [];
    }

    var z = zones.selectAll('polygon').data(data).join('polygon');
    z.call(dragAll);
    z.attrs({
      points: (d) => (d === activeObject ? constructPoints : d.points)
    });

    var p = points.selectAll('circle').data(constructPoints).join('circle');
    p.attrs({
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
      forbiddenZones.classed('active', true);
    } else {
      forbiddenZones.classed('active', false);
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
    enableEvents: enableEvents
  };
}
