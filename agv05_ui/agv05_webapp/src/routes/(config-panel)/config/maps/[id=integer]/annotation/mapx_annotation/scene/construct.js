/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import EditMode from './edit-mode';
import TextAnnotation from '../../map_annotation/models/text-annotation';
import PolygonAnnotation from '../../map_annotation/models/polygon-annotation';
import IconAnnotation from '../../map_annotation/models/icon-annotation';

var PMode = {
  NORMAL: 0,
  CLONE: 1,
  ADD: 2,
  REMOVE: 3
};

export default function (viz, scene) {
  /* Objects under construction */
  var $viz = $(viz.node());
  var Utils = scene.Utils;
  var construct = scene.append('g').attr('class', 'construct');
  var pa,
    ia,
    r,
    dg,
    cln = [];
  var pMode = PMode.NORMAL;

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

  construct.polygonAnnotation = construct.append('g').attrs({
    class: 'polygon-annotations'
  });
  construct.polygonAnnotation.polygon = construct.polygonAnnotation.append('polygon');

  construct.iconAnnotations = construct.append('g').attrs({
    class: 'icon-annotations'
  });
  construct.iconAnnotation = construct.append('use').attrs({
    class: 'icon-annotation active',
    display: 'none'
  });

  construct.textAnnotations = construct.append('g').attrs({
    class: 'text-annotations'
  });

  construct.selectionArea = construct.append('g').attrs({
    class: 'selection-area'
  });
  construct.selectionArea.polygon = construct.selectionArea.append('polygon');

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered(e) {
    /* jshint validthis: true */
    ia = null;
    updatePMode(e);
    if (scene.mode >= EditMode.LIFT) {
      let ob = Utils.snapToIconAnnotation(this.mouse);
      if (ob) {
        ia = null;
        viz.style('cursor', 'not-allowed');
      } else {
        ia = Utils.makeIconAnnotation(Utils.snap(this.mouse));
        ia.type = getIconType();
        viz.style('cursor', 'crosshair');
      }
    } else if (scene.mode === EditMode.TEXT) {
      let ob = Utils.snapToTextAnnotation(this.mouse);
      if (ob) {
        viz.style('cursor', 'not-allowed');
      } else {
        viz.style('cursor', 'text');
      }
    } else if (scene.mode === EditMode.RULER) {
      viz.style('cursor', 'crosshair');
    } else if (
      scene.mode === EditMode.POINTER &&
      (pMode === PMode.NORMAL || pMode === PMode.CLONE)
    ) {
      let ob = Utils.snapToObject(this.mouse);
      scene.hoverObject.set(ob);
      if (ob && scene.activeObject.contain(ob)) {
        viz.style('cursor', pMode === PMode.CLONE ? 'copy' : 'move');
      } else if (ob) {
        viz.style('cursor', 'pointer');
      } else {
        viz.style('cursor', 'default');
      }
    } else if (scene.mode === EditMode.POINTER) {
      // pMode ADD / REMOVE
      let ob = Utils.snapToObject(this.mouse);
      if (pMode === PMode.ADD) {
        scene.hoverObject.set(ob);
        scene.discardObject.set(null);
      } else {
        scene.discardObject.set(ob);
        scene.hoverObject.set(null);
      }
      if (ob) {
        viz.style('cursor', 'pointer');
      } else {
        viz.style('cursor', 'crosshair');
      }
    }
    drawIconAnnotation(ia);
  }

  function hoverended() {
    drawIconAnnotation();
    scene.hoverObject.set(null);
    scene.discardObject.set(null);
  }

  /* Track event */
  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);

  function trackstarted(e) {
    /* jshint validthis: true */
    updatePMode(e);
    if (scene.mode >= EditMode.LIFT) {
      e.stopPropagation();
      viz.call(noHover);

      let ob = Utils.snapToIconAnnotation(this.p0);
      if (!ob) {
        ia = Utils.makeIconAnnotation(Utils.snap(this.p0));
        ia.type = getIconType();
        ia.valid = true;
        let idx = viz.models.addIconAnnotation(ia);
        scene.activeObject.set(viz.models.rawIconAnnotations()[idx]);
      }
      drawIconAnnotation(ia);
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.TEXT) {
      e.stopPropagation();
      viz.call(noHover);

      let ob = Utils.snapToTextAnnotation(this.p0);
      if (!ob) {
        let ta = Utils.makeTextAnnotation(Utils.snap(this.p0));
        ta.valid = true;
        ta.content = 'Add text';
        let idx = viz.models.addTextAnnotation(ta);
        scene.activeObject.set(viz.models.rawTextAnnotations()[idx]);
      }
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.POLYGON) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = Utils.snap(this.p0);
      pa = Utils.makePolygonAnnotation(this._p0);
      if (pa) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
      drawPolygonAnnotation(pa);
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
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.NORMAL) {
      let ob = Utils.snapToObject(this.p0);
      if (!ob) {
        return;
      }
      if (!scene.activeObject.contain(ob)) {
        scene.activeObject.set(ob);
      }
      let ao = scene.activeObject.getAll();
      e.stopPropagation();
      viz.call(noHover);
      viz.zoom.disablePan();
      dg = {
        ctx: ao,
        cp: ob
      };
      viz.style('cursor', 'move');
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.CLONE) {
      let ob = Utils.snapToObject(this.p0);
      if (ob && scene.activeObject.contain(ob)) {
        let ao = scene.activeObject.getAll();
        e.stopPropagation();
        viz.call(noHover);
        viz.zoom.disablePan();
        cln = {
          ctx: ao,
          cp: ob
        };
        viz.style('cursor', 'copy');
        scene.activeObject.set(null);
        scene.hoverObject.set(null);
      } else if (ob) {
        scene.activeObject.set(ob);
      }
    } else if (scene.mode === EditMode.POINTER) {
      // pMode ADD / REMOVE
      e.stopPropagation();
      viz.call(noHover);
      viz.zoom.disablePan();
      let ob = Utils.snapToObject(this.p0);
      if (pMode === PMode.ADD) {
        scene.hoverObject.set(ob);
      } else {
        scene.discardObject.set(ob);
      }
      this._p0 = viz.zoom.pixelToCoord(this.p0);
      let area = Utils.makeSelectionArea(this._p0);
      viz.style('cursor', 'crosshair');
      drawSelectionArea(area);
    }
  }

  function tracked() {
    /* jshint validthis: true */
    if (!this.p1) {
      return;
    }
    this._p1 = Utils.snap(this.p1);

    if (scene.mode === EditMode.POLYGON) {
      if (!pa) {
        return;
      }
      pa.valid = false;
      if (Utils.withinCorners(this._p1)) {
        pa.valid = Utils.makePolygonAnnotation(this._p0, this._p1);
        if (pa.valid) {
          pa = pa.valid;
          pa.valid = true;
          viz.style('cursor', 'crosshair');
          drawPolygonAnnotation(pa);
        } else {
          viz.style('cursor', 'not-allowed');
        }
      }
    } else if (scene.mode === EditMode.RULER) {
      this._p1 = viz.zoom.pixelToCoord(this.p1);
      if (Utils.withinCorners(this._p1)) {
        r[1] = {
          x: this._p1[0],
          y: this._p1[1]
        };
        drawRuler(r);
      }
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.NORMAL) {
      if (dg) {
        dg.valid = false;
        if (Utils.withinCorners(this._p1)) {
          dg.valid = Utils.makeDraggables(this._p1, dg);
          if (dg.valid) {
            viz.style('cursor', 'move');
            drawDraggables(dg);
          } else {
            viz.style('cursor', 'not-allowed');
          }
        }
      }
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.CLONE) {
      if (cln) {
        cln.valid = false;
        if (Utils.withinCorners(this._p1)) {
          cln.valid = Utils.makeClone(this._p1, cln);
          if (cln.valid) {
            viz.style('cursor', 'copy');
            drawClone(cln);
          } else {
            viz.style('cursor', 'not-allowed');
          }
        }
      }
    } else if (scene.mode === EditMode.POINTER) {
      // pMode ADD / REMOVE
      this._p1 = viz.zoom.pixelToCoord(this.p1);
      // TODO: crawl the scene
      if (!Utils.withinCorners(this._p1)) {
        return;
      }

      let obs = Utils.objectsInArea(this._p0, this._p1);
      if (pMode === PMode.ADD) {
        scene.hoverObject.set(obs);
      } else {
        scene.discardObject.set(obs);
      }

      let area = Utils.makeSelectionArea(this._p0, this._p1);
      viz.style('cursor', 'crosshair');
      drawSelectionArea(area);
    }
  }

  function trackended() {
    /* jshint validthis: true */
    tracked.call(this);
    if (scene.mode === EditMode.POLYGON) {
      if (pa && pa.valid) {
        let idx = viz.models.addPolygonAnnotation(pa);
        scene.activeObject.set(viz.models.rawPolygonAnnotations()[idx]);
      }
      pa = null;
      drawPolygonAnnotation();
      sendEditMode(EditMode.POLYGON);
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.NORMAL) {
      if (dg && dg.valid) {
        scene.activeObject.set(Utils.saveDraggables(dg));
      }
      dg = null;
      drawDraggables();
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.CLONE) {
      if (cln && cln.valid) {
        scene.activeObject.set(Utils.saveClone(cln));
      }
      cln = null;
      drawClone();
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.POINTER) {
      // pMode ADD / REMOVE
      if (pMode === PMode.ADD) {
        let obs = scene.hoverObject.getAll();
        scene.hoverObject.set(null);
        scene.activeObject.add(obs);
      } else if (pMode === PMode.REMOVE) {
        let obs = scene.discardObject.getAll();
        scene.discardObject.set(null);
        scene.activeObject.discard(obs);
      }

      drawSelectionArea();
      viz.style('cursor', 'default');
      sendEditMode(EditMode.POINTER);
    }
  }

  function getIconType() {
    let type = 'lift';
    switch (scene.mode) {
      case EditMode.ROBOTARM:
        type = 'robotarm';
        break;
      case EditMode.CONVEYOR:
        type = 'conveyor';
        break;
      default:
        break;
    }
    return type;
  }

  function drawIconAnnotation(ia) {
    if (!ia) {
      construct.iconAnnotation.attr('display', 'none');
      return;
    }
    construct.iconAnnotation
      .attrs({
        x: ia.x,
        y: ia.y,
        display: 'block',
        'xlink:href': `#${ia.type}`
      })
      .classed('dragged', ia.ob);
  }

  function drawPolygonAnnotation(pa) {
    if (!pa || !pa.points || !pa.points.length || pa.points.length < 3) {
      construct.polygonAnnotation.polygon.attr('display', 'none');
      return;
    }
    construct.polygonAnnotation.polygon.attrs({
      points: pa.points,
      display: 'block',
      fill: pa.fill || '#00f',
      'fill-opacity': pa.fillOpacity || 0.2
    });
  }

  function drawRuler(r) {
    if (!Array.isArray(r)) {
      r = [];
    }

    var d = Utils.generateRulerPath(r);
    construct.ruler.attr('d', d[0]);
    construct.rulerMark.attr('d', d[1]);
    construct.rulerMarkSmall.attr('d', d[2]);

    var j = construct.rulerEndpoints.selectAll('use').data(r).join('use');
    j.attrs({
      class: 'ruler-endpoint',
      'xlink:href': '#ruler-endpoint',
      x: (d) => d.x,
      y: (d) => d.y
    });

    var t = construct.rulerLabels.selectAll('text').data(d[3]).join('text');
    t.attrs({
      class: 'ruler-label'
    });
    t.text((d) => (d.distance ? `${+d.distance.toFixed(2)} m` : '0'));
    t.attr(
      'transform',
      (d) => `translate(${d.x},${d.y})scale(1,-1)rotate(90)rotate(${d.rotation})`
    );
  }

  function drawSelectionArea(area) {
    if (!area || !area.points || !area.points.length || area.points.length < 3) {
      construct.selectionArea.polygon.attr('display', 'none');
      return;
    }
    construct.selectionArea.polygon.attrs({
      points: area.points,
      display: 'block'
    });
  }

  function drawDraggables(dg) {
    if (!dg || !dg.valid) {
      dg = {};
      dg.mctx = [];
    }
    let draggableTextAnnotations = dg.mctx.filter((ob) => ob instanceof TextAnnotation);
    drawDraggableTextAnnotations(draggableTextAnnotations);
    let draggableIconAnnotations = dg.mctx.filter((ob) => ob instanceof IconAnnotation);
    drawDraggableIconAnnotations(draggableIconAnnotations);
  }

  function drawClone(cln) {
    if (!cln || !cln.valid) {
      cln = {};
    }
    drawDraggableTextAnnotations(cln.textAnnotations);
    drawDraggableIconAnnotations(cln.iconAnnotations);
  }

  function drawDraggableTextAnnotations(draggableTextAnnotations) {
    let processedData = [];
    for (let d of draggableTextAnnotations || []) {
      let fontSize = d.size;
      let lines = d.content ? d.content.split('\n') : [];
      let baseTextX = d.x;
      let textY = d.y;
      let sub = [];
      for (let i = 0; i < lines.length; ++i) {
        let textX = baseTextX - i * fontSize * 1.2;
        sub.push({
          content: lines[i],
          size: `${d.size}pt`,
          x: textX,
          y: textY,
          parent: d
        });
      }
      processedData.push(sub);
    }

    var vizTextAnnotation = construct.textAnnotations
      .selectAll('g')
      .data(processedData || [])
      .join('g');
    var t = vizTextAnnotation
      .selectAll('text')
      .data((d) => d)
      .join('text');
    t.text((d) => d.content);
    t.attrs({
      class: 'text-annotation active',
      x: (d) => d.y * -1,
      y: (d) => d.x * -1,
      'font-size': (d) => d.size,
      transform: 'rotate(90) scale(-1,1)'
    });
  }

  function drawDraggableIconAnnotations(draggableIconAnnotations) {
    let vizIconAnnotation = construct.iconAnnotations
      .selectAll('use')
      .data(draggableIconAnnotations || [])
      .join('use');
    vizIconAnnotation.attrs({
      class: 'icon-annotation active',
      x: (d) => d.x,
      y: (d) => d.y,
      'xlink:href': (d) => `#${d.type}`
    });
  }

  function sendEditMode(m) {
    scene.mode = m;
    $viz.trigger('scene.editMode');
  }

  function setEditMode(m) {
    viz.call(hover);
    viz.call(track);
    viz.style('cursor', 'default');
    viz.on('mouseenter.hover').call(viz.node()); // Hack to reenable hover events without mouse exiting svg first.

    if (m === EditMode.TEXT) {
      viz.style('cursor', 'text');
    }

    var textAnnotation = scene.activeObject.get() instanceof TextAnnotation;
    var iconAnnotation = scene.activeObject.get() instanceof IconAnnotation;
    var polygonAnnotation = scene.activeObject.get() instanceof PolygonAnnotation;

    if (m !== EditMode.POLYGON) {
      if (polygonAnnotation) {
        scene.activeObject.set(null);
      }
    }

    if (m !== EditMode.POINTER) {
      if (!textAnnotation && !polygonAnnotation && !iconAnnotation) {
        scene.activeObject.set(null);
      }
      scene.hoverObject.set(null);
    }
  }

  function onKeyup(event) {
    if (event.keyCode === 0x10) {
      // Shift
      if (event.ctrlKey) {
        pMode = PMode.ADD;
      } else {
        pMode = PMode.NORMAL;
      }
    } else if (event.keyCode === 0x11) {
      // Ctrl
      if (event.shiftKey) {
        pMode = PMode.CLONE;
      } else {
        pMode = PMode.NORMAL;
      }
    } else if (event.keyCode === 0x13) {
      // Enter
    }
  }
  $(document.body).on('keyup', onKeyup);

  function onKeydown(event) {
    if (event.keyCode === 0x10) {
      // Shift
      if (event.ctrlKey) {
        pMode = PMode.REMOVE;
      } else {
        pMode = PMode.CLONE;
      }
    } else if (event.keyCode === 0x11) {
      // Ctrl
      if (event.shiftKey) {
        pMode = PMode.REMOVE;
      } else {
        pMode = PMode.ADD;
      }
    }
  }
  $(document.body).on('keydown', onKeydown);

  function updatePMode(e) {
    if (e.ctrlKey) {
      pMode = e.shiftKey ? PMode.REMOVE : PMode.ADD;
    } else {
      pMode = e.shiftKey ? PMode.CLONE : PMode.NORMAL;
    }
  }

  return {
    polygonAnnotation: construct.polygonAnnotation,
    setEditMode: setEditMode,

    destroy() {
      $(document.body).off('keyup', onKeyup);
      $(document.body).off('keydown', onKeydown);
    }
  };
}
