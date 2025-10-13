/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import EditMode from './edit-mode';
import Junction from '../models/junction';
import NoRotateZone from '../models/no-rotate-zone';
import LocHintZone from '../models/loc-hint-zone';
import Station from '../models/station';

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
  var j1,
    j2,
    p,
    st,
    dg,
    cln,
    fz,
    r = [];
  var pMode = PMode.NORMAL;

  construct.ruler = construct.append('path').attr('class', 'ruler');
  construct.rulerMark = construct.append('path').attr('class', 'ruler-mark');
  construct.rulerMarkSmall = construct.append('path').attr('class', 'ruler-mark-small');
  construct.rulerEndpoints = construct.append('g').attr('class', 'ruler-endpoints');
  construct.rulerLabels = construct.append('g').attr('class', 'ruler-labels');
  construct.noRotateZone = construct.append('g').attr('class', 'no-rotate-zones active');
  construct.noRotateZone.polygon = construct.noRotateZone.append('polygon');

  construct.locHintZone = construct.append('g').attrs({
    class: 'loc-hint-zones active'
  });
  construct.locHintZone.polygon = construct.locHintZone.append('polygon');

  construct.selectionArea = construct.append('g').attr('class', 'selection-area');
  construct.selectionArea.polygon = construct.selectionArea.append('polygon');

  construct.connectors = construct.append('g');
  construct.connector = construct
    .append('path')
    .attr('class', 'path active')
    .attr('display', 'none');
  construct.initial = construct
    .append('use')
    .attr('class', 'junction active')
    .attr('xlink:href', '#junction')
    .attr('display', 'none');
  construct.final = construct
    .append('use')
    .attr('class', 'junction active')
    .attr('xlink:href', '#junction')
    .attr('display', 'none');
  construct.drag = construct.append('g');
  construct.stations = construct.append('g');
  construct.station = construct
    .append('use')
    .attr('class', 'station active')
    .attr('xlink:href', '#station')
    .attr('display', 'none');

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered(e) {
    /* jshint validthis: true */
    j1 = null;
    st = null;
    updatePMode(e);
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT) {
      j1 = Utils.makeJunction(Utils.snap(this.mouse));
      if (j1) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
    } else if (scene.mode === EditMode.STATION) {
      j1 = Utils.snapToJunction(this.mouse);
      if (j1) {
        st = Utils.makeStation(j1);
      }
      if (j1 && st) {
        viz.style('cursor', 'crosshair');
      } else {
        j1 = null;
        viz.style('cursor', 'not-allowed');
      }
    } else if (scene.mode === EditMode.NO_ROTATE_ZONE) {
      console.log('Error');
    } else if (scene.mode === EditMode.LOC_HINT_ZONE) {
      console.log('Error');
    } else if (scene.mode === EditMode.RULER) {
      viz.style('cursor', 'crosshair');
    } else if (
      scene.mode === EditMode.POINTER &&
      (pMode === PMode.NORMAL || pMode === PMode.CLONE)
    ) {
      let ob = Utils.snapToObject(this.mouse);
      scene.hoverObject.set(ob);
      if (ob instanceof Junction && scene.activeObject.contain(ob)) {
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
    drawConnector(j1);
    drawStation(st, j1);
  }

  function hoverended() {
    drawConnector();
    drawStation();
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
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = Utils.snap(this.p0);
      j1 = Utils.makeJunction(this._p0);
      if (j1) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
      drawConnector(j1);
    } else if (scene.mode === EditMode.STATION) {
      e.stopPropagation();

      j1 = Utils.snapToJunction(this.p0);
      if (j1) {
        st = Utils.makeStation(j1);
      }
      if (j1 && st) {
        let idx = viz.models.addStation(st, j1);
        scene.activeObject.set(viz.models.rawStations()[idx]);
      }
      drawConnector();
      drawStation();
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.NO_ROTATE_ZONE) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = Utils.snap(this.p0);
      fz = Utils.makeArea(this._p0);
      if (fz) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
      drawNoRotateZone(fz);
    } else if (scene.mode === EditMode.LOC_HINT_ZONE) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = Utils.snap(this.p0);
      fz = Utils.makeArea(this._p0);
      if (fz) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
      drawLocHintZone(fz);
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
      if (ob && ob instanceof Junction) {
        if (!scene.activeObject.contain(ob)) {
          scene.activeObject.set(ob);
        }
        // draggable junctions
        let ao = scene.activeObject.getAll();
        let junctions = viz.models.lookupJunctionsFromObjects(ao);
        e.stopPropagation();
        viz.call(noHover);
        viz.zoom.disablePan();

        dg = {
          ctx: ao,
          cp: ob,
          obs: junctions
        };
        viz.style('cursor', 'move');
      } else if (ob) {
        scene.activeObject.set(ob);
      }
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.CLONE) {
      let ob = Utils.snapToObject(this.p0);
      if (ob && ob instanceof Junction && scene.activeObject.contain(ob)) {
        let ao = scene.activeObject.getAll();
        let junctions = viz.models.lookupJunctionsFromObjects(ao);
        e.stopPropagation();
        viz.call(noHover);
        viz.zoom.disablePan();
        cln = {
          ctx: ao,
          cp: ob,
          obs: junctions
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
      let area = Utils.makeArea(this._p0);
      viz.style('cursor', 'crosshair');
      drawSelectionArea(area);
    }
  }

  function tracked() {
    /* jshint validthis: true */
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT) {
      j2 = null;
      if (!j1 || !this.p1) {
        return;
      }
      this._p1 = Utils.snap(this.p1);

      if (Utils.withinCorners(this._p1)) {
        this._p1 = Utils.limitMovement(this._p0, this._p1);
        j2 = Utils.makeJunction(this._p1);
        p = Utils.makePath(j1, j2, scene.mode);

        if (j2 && p) {
          if (p.distance > 0.599) {
            viz.style('cursor', 'crosshair');
          } else {
            viz.style('cursor', 'not-allowed');
          }
          drawConnector(j1, j2, p);
        } else {
          viz.style('cursor', 'not-allowed');
        }
      }
    } else if (scene.mode === EditMode.NO_ROTATE_ZONE) {
      if (!fz || !this.p1) {
        return;
      }
      this._p1 = Utils.snap(this.p1);

      fz.valid = false;
      if (Utils.withinCorners(this._p1)) {
        fz.valid = Utils.makeArea(this._p0, this._p1);

        if (fz.valid) {
          fz = fz.valid;
          fz.valid = true;
          viz.style('cursor', 'crosshair');
          drawNoRotateZone(fz);
        } else {
          viz.style('cursor', 'not-allowed');
        }
      }
    } else if (scene.mode === EditMode.LOC_HINT_ZONE) {
      if (!fz || !this.p1) {
        return;
      }
      this._p1 = Utils.snap(this.p1);

      fz.valid = false;
      if (Utils.withinCorners(this._p1)) {
        fz.valid = Utils.makeArea(this._p0, this._p1);

        if (fz.valid) {
          fz = fz.valid;
          fz.valid = true;
          viz.style('cursor', 'crosshair');
          drawLocHintZone(fz);
        } else {
          viz.style('cursor', 'not-allowed');
        }
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
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.NORMAL) {
      if (!this.p1) {
        return;
      }
      this._p1 = Utils.snap(this.p1);

      // draggable junctions
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
      if (!this.p1) {
        return;
      }
      this._p1 = Utils.snap(this.p1);
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
      if (!this.p1) {
        return;
      }
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

      let area = Utils.makeArea(this._p0, this._p1);
      viz.style('cursor', 'crosshair');
      drawSelectionArea(area);
    }
  }

  function trackended() {
    /* jshint validthis: true */
    tracked.call(this);
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT) {
      if (j1 && j2 && p && p.distance > 0.599) {
        let idx = viz.models.addPath(p, j1, j2);
        scene.activeObject.set(viz.models.rawPaths()[idx]);
      }
      j1 = j2 = p = null;
      drawConnector();
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.NO_ROTATE_ZONE) {
      if (fz && fz.valid) {
        let idx = viz.models.addNoRotateZone(fz);
        scene.activeObject.set(viz.models.rawNoRotateZones()[idx]);
      }
      fz = null;
      drawNoRotateZone();
      sendEditMode(EditMode.NO_ROTATE_ZONE);
    } else if (scene.mode === EditMode.LOC_HINT_ZONE) {
      if (fz && fz.valid) {
        let idx = viz.models.addLocHintZone(fz);
        scene.activeObject.set(viz.models.rawLocHintZones()[idx]);
      }
      fz = null;
      drawLocHintZone();
      sendEditMode(EditMode.LOC_HINT_ZONE);
    } else if (scene.mode === EditMode.POINTER && pMode === PMode.NORMAL) {
      // draggable junctions
      if (dg) {
        if (dg.valid) {
          scene.activeObject.set(Utils.saveDraggables(dg));
        }
        dg = null;
        drawDraggables();
        sendEditMode(EditMode.POINTER);
      }
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

  function drawConnector(j1, j2, p) {
    if (!j1) {
      construct.initial.attr('display', 'none');
      construct.final.attr('display', 'none');
      construct.connector.attr('display', 'none');
      return;
    }

    construct.initial.attr('x', j1.x).attr('y', j1.y).attr('display', 'block');

    if (!j2) {
      construct.final.attr('display', 'none');
      construct.connector.attr('display', 'none');
      return;
    }

    construct.final.attr('x', j2.x).attr('y', j2.y).attr('display', 'block');

    if (!p) {
      construct.connector.attr('display', 'none');
      return;
    }

    construct.connector
      .attr('d', Utils.generateConnectorPath(p, j1, j2))
      .attr('display', 'block');
  }

  function drawStation(st, j1) {
    if (!st || !j1) {
      construct.station.attr('display', 'none');
      return;
    }
    construct.station
      .attr('transform', `translate(${j1.x},${j1.y})rotate(${st.direction * 90})`)
      .attr(
        'xlink:href',
        st.direction === Station.Direction.NA ? '#station-directionless' : '#station'
      )
      .attr('display', 'block');
  }

  function drawDraggables(dg) {
    if (!dg) {
      dg = {};
    }

    var vizJ = construct.drag
      .selectAll('use')
      .data(dg.j || [])
      .join('use');
    vizJ
      .attr('class', 'junction active')
      .attr('xlink:href', '#junction')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y);

    var p = (dg.p || []).concat(dg.pp || []);
    var vizP = construct.connectors.selectAll('path.path').data(p).join('path');
    vizP
      .attr('class', 'path active')
      .attr('d', (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1]));

    var vizSt = construct.stations
      .selectAll('use')
      .data(dg.st || [])
      .join('use');
    vizSt.attrs({
      class: 'station active',
      transform: (d) =>
        `translate(${dg.j[d[0]].x},${dg.j[d[0]].y})rotate(${d[1].direction * 90})`,
      'xlink:href': (d) =>
        d[1].direction === Station.Direction.NA ? '#station-directionless' : '#station'
    });
  }

  function drawNoRotateZone(fz) {
    if (!fz || !fz.points || !fz.points.length || fz.points.length < 3) {
      construct.noRotateZone.polygon.attr('display', 'none');
      return;
    }
    construct.noRotateZone.polygon.attr('points', fz.points).attr('display', 'block');
  }

  function drawLocHintZone(fz) {
    if (!fz || !fz.points || !fz.points.length || fz.points.length < 3) {
      construct.locHintZone.polygon.attr('display', 'none');
      return;
    }
    construct.locHintZone.polygon.attrs({
      points: fz.points,
      display: 'block'
    });
  }

  function drawSelectionArea(area) {
    if (!area || !area.points || !area.points.length || area.points.length < 3) {
      construct.selectionArea.polygon.attr('display', 'none');
      return;
    }
    construct.selectionArea.polygon.attr('points', area.points).attr('display', 'block');
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
    j.attr('class', 'ruler-endpoint')
      .attr('xlink:href', '#ruler-endpoint')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y);

    var t = construct.rulerLabels.selectAll('text').data(d[3]).join('text');
    t.attr('class', 'ruler-label')
      .text((d) => (d.distance ? `${+d.distance.toFixed(2)} m` : '0'))
      .attr('transform', (d) => `translate(${d.x},${d.y})rotate(${d.rotation})`);
  }

  function drawClone(cln) {
    if (!cln || !cln.valid) {
      cln = {};
    }

    var vizJ = construct.drag
      .selectAll('use')
      .data(cln.j || [])
      .join('use');
    vizJ
      .attr('class', 'junction active')
      .attr('xlink:href', '#junction')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y);

    var vizP = construct.connectors
      .selectAll('path.path')
      .data(cln.p || [])
      .join('path');
    vizP
      .attr('class', 'path active')
      .attr('d', (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1]));

    var vizSt = construct.stations
      .selectAll('use')
      .data(cln.st || [])
      .join('use');
    vizSt.attrs({
      class: 'station active',
      transform: (d) =>
        `translate(${cln.j[d[0]].x},${cln.j[d[0]].y})rotate(${d[1].direction * 90})`,
      'xlink:href': (d) =>
        d[1].direction === Station.Direction.NA ? '#station-directionless' : '#station'
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

    var noRotateZone = scene.activeObject.get() instanceof NoRotateZone;
    if (m !== EditMode.NO_ROTATE_ZONE) {
      if (noRotateZone) {
        scene.activeObject.set(null);
      }
    }
    var locHintZone = scene.activeObject.get() instanceof LocHintZone;
    if (m !== EditMode.LOC_HINT_ZONE) {
      if (locHintZone) {
        scene.activeObject.set(null);
      }
    }
    if (m !== EditMode.POINTER) {
      if (!noRotateZone && !locHintZone) {
        scene.activeObject.set(null);
      }
      scene.hoverObject.set(null);
    }
  }

  function updatePMode(e) {
    if (e.ctrlKey) {
      pMode = e.shiftKey ? PMode.REMOVE : PMode.ADD;
    } else {
      pMode = e.shiftKey ? PMode.CLONE : PMode.NORMAL;
    }
  }

  return {
    noRotateZone: construct.noRotateZone,
    locHintZone: construct.locHintZone,
    setEditMode: setEditMode
  };
}
