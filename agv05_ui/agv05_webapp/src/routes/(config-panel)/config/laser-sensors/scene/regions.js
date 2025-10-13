/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import contextMenuF from '$lib/shared/context-menu.js';
let contextMenu = contextMenuF();

var deleteMenuHtml = `
<li><button type="button" class="remove">Remove</button></li>
`;

var _names = ['far', 'middle', 'near'];
// HACK: prevent purgecss to drop these css class names.
var _css_names = ['region-far', 'region-middle', 'region-near'];

export default function (viz, scene) {
  /* Regions */
  var $viz = $(viz.node());
  var regions = scene.append('g').attr('class', 'regions');

  var polygons = {};
  for (let n of _names) {
    polygons[n] = regions.append('polygon').attr('class', 'region-' + n);
  }

  var points = regions.append('g').attr('class', 'region-active');
  var construct = regions.append('circle').attrs({
    class: 'construct',
    display: 'none'
  });

  var activeRegion = 'near';
  var activeCoord = -1;
  var constructIdx = -1;
  var constructActive = false;

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);
  viz.call(hover);

  function hovered() {
    /* jshint validthis: true */
    var pt = viz.zoom.pixelToCoord(this.mouse);
    pt = viz.models.hitTestRegionPerimeter(activeRegion, [pt[0] * 1000, pt[1] * 1000])[0];
    drawConstruct(pt);
    pt = !!pt;
    if (constructActive !== pt) {
      constructActive = pt;
      if (constructActive) {
        viz.zoom.disablePan();
      } else {
        viz.zoom.enable();
      }
    }
  }

  function hoverended() {
    drawConstruct();
  }

  /* Drag event */
  var drag = d3.drag().on('start', dragstarted).on('drag', dragged).on('end', dragended);
  construct.call(drag);

  function dragstarted(event, d) {
    /* jshint validthis: true */
    event.sourceEvent.stopPropagation();
    drawConstruct();
    if (this === construct.node()) {
      constructIdx = viz.models.addCoord(activeRegion, d);
      setActiveCoord(constructIdx);
    } else {
      let idx = viz.models.rawRegions()[activeRegion].indexOf(d);
      setActiveCoord(idx);
    }
    regions.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragged(event) {
    var pt = viz.zoom.computeSnap([event.x, event.y]);
    viz.models.updateCoord(activeRegion, activeCoord, [pt[0] * 1000, pt[1] * 1000]);
  }

  function dragended() {
    regions.classed('drag-active', false);
    enableEvents();
  }

  var dragAll = d3
    .drag()
    .on('start', dragAllStarted)
    .on('drag', dragAllDragged)
    .on('end', dragAllEnded);

  for (let n of _names) {
    polygons[n].datum(n).call(dragAll);
  }

  function dragAllStarted(event, d) {
    /* jshint validthis: true */
    event.sourceEvent.stopPropagation();
    setActiveRegion(d);
    setActiveCoord(-1);
    var data = viz.models.rawRegions();
    var firstPt = data[activeRegion][0];
    this.translate = [firstPt[0], firstPt[1]];
    self.dragAllStartPt = [event.x, event.y];

    regions.classed('drag-active', true);
    viz.style('cursor', 'move');
    viz.call(noHover);
  }

  function dragAllDragged(event) {
    /* jshint validthis: true */
    var pt = viz.zoom.computeSnap([
      event.x - self.dragAllStartPt[0],
      event.y - self.dragAllStartPt[1]
    ]);
    viz.models.moveRegion(activeRegion, [
      Math.round(pt[0] * 1000) + this.translate[0],
      Math.round(pt[1] * 1000) + this.translate[1]
    ]);
  }

  function dragAllEnded() {
    regions.classed('drag-active', false);
    enableEvents();
  }

  /* Context menu */
  var deleteMenu = $(deleteMenuHtml);
  deleteMenu.find('.remove').on('click', function () {
    var idx = deleteMenu.data('i');
    if (Number.isInteger(idx)) {
      viz.models.removeCoord(activeRegion, idx);
    }
    contextMenu.hide();
    return false;
  });
  contextMenu.setContents(deleteMenu);

  function showDeleteMenu(event, d) {
    var data = viz.models.rawRegions();
    var dataP = [];
    if (activeRegion && _names.indexOf(activeRegion) >= 0) {
      dataP = data[activeRegion];
    }
    let idx = dataP.indexOf(d);
    if (idx >= 0) {
      deleteMenu.data('i', idx);
      contextMenu.show(event.pageX, event.pageY);
    }
  }

  function drawConstruct(pt) {
    if (!pt) {
      construct.attr('display', 'none');
      return;
    }
    construct.datum(pt).attrs({
      cx: (d) => d[0] * 0.001,
      cy: (d) => d[1] * 0.001,
      display: 'block'
    });
  }

  function modelsUpdated() {
    var data = viz.models.rawRegions();
    var dataP = [];
    if (activeRegion && _names.indexOf(activeRegion) >= 0) {
      dataP = data[activeRegion];
    }

    for (let n of _names) {
      var d = data[n].map((p) => [p[0] * 0.001, p[1] * 0.001]);
      polygons[n].attr('points', d);
    }

    var p = points.selectAll('circle').data(dataP).join('circle');
    p.call(drag).on('contextmenu', showDeleteMenu);
    p.attrs({
      r: _getRadius(),
      'stroke-width': _getStrokeWidth(),
      cx: (d) => d[0] * 0.001,
      cy: (d) => d[1] * 0.001
    });
    p.classed('active', (d, idx) => idx === activeCoord);

    $viz.trigger('scene.updateTable');
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

  function enableEvents() {
    viz.call(hover);
    viz.style('cursor', 'default');
    viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.
  }

  function setActiveRegion(name) {
    if (activeRegion === name) {
      return;
    }
    activeRegion = name;
    $viz.trigger('scene.activeRegion', activeRegion);
    modelsUpdated();
  }

  function setActiveCoord(idx) {
    if (activeCoord === idx) {
      return;
    }
    activeCoord = idx;
    $viz.trigger('scene.activeCoord', activeCoord);
    modelsUpdated();
    contextMenu.hide();
  }

  return {
    modelsUpdated: modelsUpdated,
    zoomed: zoomed,
    setActiveRegion: setActiveRegion,
    setActiveCoord: setActiveCoord
  };
}
