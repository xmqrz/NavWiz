/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as d3 from 'd3';

import EditMode from './edit-mode';

export default function (viz, scene) {
  /* Objects under construction */
  var canvas = viz.canvas;
  var Utils = scene.Utils;
  var construct = scene.append('g').attr('class', 'construct');
  var c1,
    c2,
    r = [];

  construct.ruler = construct.append('path').attrs({
    class: 'ruler'
  });
  construct.rulerMark = construct.append('path').attrs({
    class: 'ruler-mark'
  });
  construct.rulerMarkSmall = construct.append('path').attrs({
    class: 'ruler-mark-small'
  });
  construct.rulerEndpoints = construct.append('g').attrs({
    class: 'ruler-endpoints'
  });
  construct.rulerLabels = construct.append('g').attrs({
    class: 'ruler-labels'
  });

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered() {
    /* jshint validthis: true */
    clearCanvas();
    c1 = c2 = null;
    if (scene.mode >= EditMode.LINE) {
      c1 = canvas.ptFromDiv(this.mouse);
      if (canvas.withinCorners(c1)) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
        c1 = null;
      }
      drawCanvas();
    } else if (scene.mode === EditMode.RULER) {
      viz.style('cursor', 'crosshair');
    }
  }

  function hoverended() {
    clearCanvas();
  }

  /* Track event */
  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);

  function trackstarted(e) {
    /* jshint validthis: true */
    clearCanvas();
    c1 = c2 = null;
    if (scene.mode >= EditMode.LINE) {
      e.stopPropagation();
      viz.call(noHover);
      c1 = canvas.ptFromDiv(this.p0);
      if (canvas.withinCorners(c1)) {
        viz.style('cursor', 'crosshair');
        drawCanvas();
      } else {
        viz.style('cursor', 'not-allowed');
        c1 = null;
      }
    } else if (scene.mode === EditMode.RULER) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = viz.zoom.pixelToCoord(this.p0);
      r = [
        {
          x: this._p0[0],
          y: this._p0[1]
        }
      ];
      r.push(r[0]);
      drawRuler();
    }
  }

  function tracked(e) {
    /* jshint validthis: true */
    clearCanvas();
    if (scene.mode >= EditMode.LINE) {
      c2 = null;
      if (!c1) {
        return;
      }
      if (!this.p1) {
        // happens on click and release
        this.p1 = this.p0;
      }
      c2 = canvas.ptFromDiv(this.p1);
      if (canvas.withinCorners(c2) && withinCorners(this.p1)) {
        viz.style('cursor', 'crosshair');
        if (scene.mode === EditMode.LINE && e.shiftKey) {
          c2 = canvas.limitMovement(c1, c2);
        }
        drawCanvas();
      } else {
        viz.style('cursor', 'not-allowed');
        c2 = null;
      }
    } else if (scene.mode === EditMode.RULER) {
      if (!this.p1) {
        return;
      }
      this._p1 = viz.zoom.pixelToCoord(this.p1);
      if (Utils.withinCorners(this._p1)) {
        r[1] = {
          x: this._p1[0],
          y: this._p1[1]
        };
        drawRuler(r);
      }
    }
  }

  function trackended(e) {
    /* jshint validthis: true */
    tracked.call(this, e);

    if (scene.mode >= EditMode.LINE) {
      if (c1 && c2) {
        viz.models.updatePng(canvas.node());
      }
      c1 = c2 = null;
      clearCanvas();
      setEditMode(scene.mode);
    }
  }

  function withinCorners(pt) {
    return pt[0] >= 0 && pt[0] < viz.meta.width && pt[1] >= 0 && pt[1] < viz.meta.height;
  }

  function clearCanvas() {
    if (!c1) {
      return;
    }
    var args = c2 ? c1.concat(c2) : c1.concat(c1);
    canvas.renderPng.apply(null, args);
  }

  function drawCanvas() {
    if (!c1) {
      return;
    }
    if (scene.mode >= EditMode.LINE && scene.mode < EditMode.LIVE_SET_POSE) {
      var args = c2 ? c1.concat(c2) : c1.concat(c1);
      const fn = [
        canvas.drawLine,
        canvas.drawRectangle,
        canvas.drawWhiteRectangle,
        canvas.drawWhiteRectangle2,
        canvas.clearRectangle
      ];
      fn[scene.mode - EditMode.LINE].apply(null, args);
    }
  }

  function drawRuler(r) {
    if (!Array.isArray(r)) {
      r = [];
    }

    var d = Utils.generateRulerPath(r);
    construct.ruler.attr('d', d[0]);
    construct.rulerMark.attr('d', d[1]);
    construct.rulerMarkSmall.attr('d', d[2]);

    var j = construct.rulerEndpoints.selectAll('use').data(r);
    j.enter().append('use').attrs({
      class: 'ruler-endpoint',
      'xlink:href': '#ruler-endpoint'
    });
    j.attrs({
      x: (d) => d.x,
      y: (d) => d.y
    });
    j.exit().remove();

    var t = construct.rulerLabels.selectAll('text').data(d[3]);
    t.enter().append('text').attrs({
      class: 'ruler-label'
    });
    t.text((d) => (d.distance ? `${+d.distance.toFixed(2)} m` : '0'));
    t.attr(
      'transform',
      (d) => `translate(${d.x},${d.y})scale(1,-1)rotate(90)rotate(${d.rotation})`
    );
    t.exit().remove();
  }

  // function sendEditMode(m) {
  //   scene.mode = m;
  //   $viz.trigger('scene.editMode');
  // }

  function setEditMode(_m) {
    viz.call(hover);
    viz.call(track);
    viz.style('cursor', 'default');
    viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.
  }

  return {
    setEditMode: setEditMode
  };
}
