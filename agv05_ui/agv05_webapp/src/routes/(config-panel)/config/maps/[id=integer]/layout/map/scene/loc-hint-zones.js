/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import LocHintZone from '../models/loc-hint-zone';
import contextMenuF from '$lib/shared/context-menu.js';
let contextMenu = contextMenuF();

var deleteMenuHtml = `
<li><button type="button" class="remove">Remove</button></li>
`;

export default function (viz, scene, tracked = true) {
  /* Loc Hint Zones */
  var locHintZones = scene.append('g').attr('class', 'loc-hint-zones');

  var zones = locHintZones.append('g').attr('class', 'zones');
  var points = locHintZones.append('g').attr('class', 'zone-active');
  var names = locHintZones.append('g').attr('class', 'zones-name');
  var construct = locHintZones
    .append('circle')
    .attr('class', 'construct')
    .attr('display', 'none');

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
    locHintZones.classed('drag-active', true);
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
      viz.models.push('dragLocHintZoneCorner');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      constructPoints.prev = undefined;
      viz.models.updateLocHintZone(activeZone);
    } else {
      if (constructPoints.length !== activeZone.points.length) {
        // implies activeCoord === constructIdx
        activeCoord = -1;
      }
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    locHintZones.classed('drag-active', false);
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
    dragAllStartPt = [event.x, event.y];
    setActiveZone(d);
    setActiveCoord(-1);

    locHintZones.classed('drag-active', true);
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
      viz.models.push('dragLocHintZone');
      activeZone.points = constructPoints.slice();
      constructPoints.valid = false;
      viz.models.updateLocHintZone(activeZone);
    } else {
      constructPoints = activeZone.points.slice();
      modelsUpdated();
    }
    locHintZones.classed('drag-active', false);
    enableEvents();
  }

  /* Context menu */
  var deleteMenu = $(deleteMenuHtml);
  var remove = deleteMenu.find('.remove').on('click', function () {
    var newPoints = deleteMenu.data('newPoints');
    if (newPoints) {
      activeCoord = -1;
      constructPoints = newPoints;
      viz.models.push('removeLocHintZoneCorner');
      activeZone.points = constructPoints.slice();
      viz.models.updateLocHintZone(activeZone);
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
    construct
      .datum(pt)
      .attr('cx', (d) => d[0])
      .attr('cy', (d) => d[1])
      .attr('display', 'block');
  }

  function modelsUpdated() {
    var data = viz.models.rawLocHintZones();
    var activeObject = scene.activeObject.get();

    if (activeObject instanceof LocHintZone) {
      if (activeZone !== activeObject) {
        activeZone = activeObject;
        activeCoord = -1;
        constructPoints = activeZone.points.slice();
      }
    } else {
      activeZone = null;
      constructPoints = [];
    }

    for (let d of data) {
      d.center = calculateCentroid(d === activeObject ? constructPoints : d.points);
    }

    var z = zones.selectAll('polygon').data(data).join('polygon');
    z.attr('points', (d) => (d === activeObject ? constructPoints : d.points)).call(dragAll);

    var t = names.selectAll('text').data(data).join('text');
    t.text((d) => d.name)
      .classed('active', (d) => d === activeObject)
      .attrs({
        transform: (d) =>
          tracked
            ? ''
            : `rotate(90,${d.center[0]},${d.center[1]})scale(-1, 1)translate(${-(2 * d.center[0])},0)`,
        'font-size': tracked ? 0.4 : 0.1,
        x: (d) => d.center[0],
        y: (d) => d.center[1],
        'stroke-width': tracked ? 0.2 : 0.05
      });

    var p = points.selectAll('circle').data(constructPoints).join('circle');
    p.attr('r', _getRadius())
      .attr('stroke-width', _getStrokeWidth())
      .call(drag)
      .on('contextmenu', showDeleteMenu)
      .attr('cx', (d) => d[0])
      .attr('cy', (d) => d[1])
      .classed('active', (d, idx) => idx === activeCoord);
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
      locHintZones.classed('active', true);
      locHintZones.classed('hidden', false);
    } else {
      locHintZones.classed('active', false);
      locHintZones.classed('hidden', true);
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
      construct = newParent
        .append('circle')
        .attr('class', 'construct')
        .attr('display', 'none');
      construct.call(drag);
    },
    modelsUpdated: modelsUpdated,
    zoomed: zoomed,
    enableEvents: enableEvents
  };
}

function calculateCentroid(vertices) {
  let sumX = 0;
  let sumY = 0;
  let area = 0;

  const n = vertices.length;
  if (n < 3) {
    return [0, 0];
  }

  for (let i = 0; i < n; i++) {
    const x0 = vertices[i][0];
    const y0 = vertices[i][1];
    const x1 = vertices[(i + 1) % n][0]; // next vertex (wrap around)
    const y1 = vertices[(i + 1) % n][1];

    const crossProduct = x0 * y1 - x1 * y0;
    area += crossProduct;
    sumX += (x0 + x1) * crossProduct;
    sumY += (y0 + y1) * crossProduct;
  }

  area *= 0.5;

  const centroidX = sumX / (6 * area);
  const centroidY = sumY / (6 * area);

  return [centroidX, centroidY];
}
