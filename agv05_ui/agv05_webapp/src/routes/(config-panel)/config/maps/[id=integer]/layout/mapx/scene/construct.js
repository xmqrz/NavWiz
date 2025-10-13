/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';
import * as d3 from 'd3';

import EditMode from './edit-mode';
import ForbiddenZone from '../models/forbidden-zone';
import Junction from '../models/junction';
import Landmark from '../models/landmark';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import LocHintZone from 'map-layout-editor/models/loc-hint-zone';
import Path from '../models/path';
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
    lm,
    fz,
    r = [];
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

  construct.forbiddenZone = construct.append('g').attrs({
    class: 'forbidden-zones active'
  });
  construct.forbiddenZone.polygon = construct.forbiddenZone.append('polygon');

  construct.noRotateZone = construct.append('g').attrs({
    class: 'no-rotate-zones active'
  });
  construct.noRotateZone.polygon = construct.noRotateZone.append('polygon');

  construct.locHintZone = construct.append('g').attrs({
    class: 'loc-hint-zones active'
  });
  construct.locHintZone.polygon = construct.locHintZone.append('polygon');

  construct.selectionArea = construct.append('g').attrs({
    class: 'selection-area'
  });
  construct.selectionArea.polygon = construct.selectionArea.append('polygon');

  construct.landmark = construct.append('use').attrs({
    class: 'landmark landmark-lg active',
    'xlink:href': '#landmark',
    display: 'none'
  });

  construct.connectors = construct.append('g');
  construct.connector = construct.append('path').attrs({
    class: 'path active',
    display: 'none'
  });
  construct.initial = construct.append('use').attrs({
    class: 'junction active',
    'xlink:href': '#junction',
    display: 'none'
  });
  construct.final = construct.append('use').attrs({
    class: 'junction active',
    'xlink:href': '#junction',
    display: 'none'
  });
  construct.drag = construct.append('g');
  construct.stations = construct.append('g');
  construct.station = construct.append('use').attrs({
    class: 'station active',
    'xlink:href': '#station',
    display: 'none'
  });
  construct.bezier = construct.append('path').attrs({
    class: 'bezier active',
    display: 'none'
  });
  construct.bezierCP = construct.append('g').attrs({
    class: 'bezier-cp',
    display: 'none'
  });
  construct.bezierCP.lines = [
    construct.bezierCP.append('line'),
    construct.bezierCP.append('line')
  ];
  construct.bezierCP.cp1 = construct.bezierCP.append('use').attrs({
    'xlink:href': '#bezier-cp'
  });
  construct.bezierCP.cp2 = construct.bezierCP.append('use').attrs({
    'xlink:href': '#bezier-cp'
  });

  /* Hover event */
  var noHover = d3.hover(false);
  var hover = d3.hover(true).on('hover', hovered).on('hoverend', hoverended);

  function hovered(e) {
    /* jshint validthis: true */
    j1 = null;
    st = null;
    lm = null;
    updatePMode(e);
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT && scene.mode < EditMode.LIVE_SET_POSE) {
      j1 = Utils.makeJunction(this.mouse, !e.altKey);
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
    } else if (scene.mode === EditMode.LANDMARK) {
      let ob = Utils.snapToLandmark(this.mouse);
      scene.hoverObject.set(ob);
      if (ob) {
        viz.style('cursor', 'pointer');
      } else {
        if (!e.altKey) {
          this._mouse = Utils.snap(this.mouse);
        } else {
          this._mouse = viz.zoom.pixelToCoord(this.mouse);
        }
        lm = Utils.makeLandmark(this._mouse);
        viz.style('cursor', 'crosshair');
      }
    } else if (scene.mode === EditMode.FORBIDDEN_ZONE) {
      console.log('Error');
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
      let ob = scene.activeObject.get();
      let hitCP = Utils.snapToBezierCP(this.mouse, ob);
      drawBezierCP(ob, hitCP);

      if (hitCP) {
        viz.style('cursor', 'move');
      } else {
        ob = Utils.snapToObject(this.mouse);
        scene.hoverObject.set(ob);
        if (ob instanceof Junction && scene.activeObject.contain(ob)) {
          viz.style('cursor', pMode === PMode.CLONE ? 'copy' : 'move');
        } else if (ob) {
          viz.style('cursor', 'pointer');
        } else {
          viz.style('cursor', 'default');
        }
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
    drawLandmark(lm);
  }

  function hoverended() {
    drawConnector();
    drawStation();
    drawBezierCP(scene.activeObject.get());
    drawLandmark();
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
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT && scene.mode < EditMode.LIVE_SET_POSE) {
      e.stopPropagation();
      viz.call(noHover);
      j1 = Utils.makeJunction(this.p0, !e.altKey);
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
    } else if (scene.mode === EditMode.LANDMARK) {
      e.stopPropagation();
      viz.call(noHover);

      let ob = Utils.snapToLandmark(this.p0);
      scene.activeObject.set(ob);
      if (ob) {
        lm = new Landmark(ob);
        lm.ob = ob;
      } else {
        if (!e.altKey) {
          this._p0 = Utils.snap(this.p0);
        } else {
          this._p0 = viz.zoom.pixelToCoord(this.p0);
        }
        lm = Utils.makeLandmark(this._p0);
        lm.valid = true;
      }
      viz.style('cursor', 'move');
      drawLandmark(lm);
    } else if (scene.mode === EditMode.FORBIDDEN_ZONE) {
      e.stopPropagation();
      viz.call(noHover);
      this._p0 = Utils.snap(this.p0);
      fz = Utils.makeArea(this._p0);
      if (fz) {
        viz.style('cursor', 'crosshair');
      } else {
        viz.style('cursor', 'not-allowed');
      }
      drawForbiddenZone(fz);
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
    } else if (
      scene.mode === EditMode.POINTER &&
      (pMode === PMode.NORMAL || pMode === PMode.CLONE)
    ) {
      // draggable control points for bezier curve
      if (!e.altKey) {
        this._p0 = Utils.snap(this.p0);
      } else {
        this._p0 = viz.zoom.pixelToCoord(this.p0);
      }
      let ob = scene.activeObject.get();
      let hitCP = Utils.snapToBezierCP(this.p0, ob);
      if (hitCP) {
        e.stopPropagation();
        viz.call(noHover);
        viz.zoom.disablePan();

        p = new Path(ob);
        p.ob = ob;
        p.hitCP = hitCP;
        if (p.hitCP === 1) {
          [p.cp1.x, p.cp1.y] = this._p0;
        } else {
          [p.cp2.x, p.cp2.y] = this._p0;
        }
        j1 = viz.models.rawJunctions()[p.j1];
        j2 = viz.models.rawJunctions()[p.j2];
        viz.style('cursor', 'move');
        drawBezierCP(p, p.hitCP);
      } else if (pMode === PMode.CLONE) {
        ob = Utils.snapToObject(this.p0);
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
          drawBezierCP(ob);
        }
      } else {
        ob = Utils.snapToObject(this.p0);
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
          drawBezierCP(ob);
        }
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

  function tracked(e) {
    /* jshint validthis: true */
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT && scene.mode < EditMode.LIVE_SET_POSE) {
      j2 = null;
      if (!j1 || !this.p1) {
        return;
      }
      if (Utils.withinCorners(viz.zoom.pixelToCoord(this.p1))) {
        j2 = Utils.makeJunction(this.p1, !e.altKey);
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
    } else if (scene.mode === EditMode.LANDMARK) {
      // draggable landmarks
      if (!lm || !this.p1) {
        return;
      }
      if (!e.altKey) {
        this._p1 = Utils.snap(this.p1);
      } else {
        this._p1 = viz.zoom.pixelToCoord(this.p1);
      }

      lm.valid = false;
      if (Utils.withinCorners(this._p1)) {
        lm.valid = Utils.makeLandmark(this._p1, lm.ob);
        if (lm.valid) {
          lm.x = lm.valid.x;
          lm.y = lm.valid.y;
          viz.style('cursor', 'move');
          drawLandmark(lm);
        } else {
          viz.style('cursor', 'not-allowed');
        }
      }
    } else if (scene.mode === EditMode.FORBIDDEN_ZONE) {
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
          drawForbiddenZone(fz);
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

      if (!e.altKey) {
        this._p1 = Utils.snap(this.p1);
      } else {
        this._p1 = viz.zoom.pixelToCoord(this.p1);
      }

      // draggable control points for bezier curve
      if (p && p.hitCP) {
        p.valid = false;
        if (Utils.withinCorners(this._p1)) {
          let cp = {
            x: this._p1[0],
            y: this._p1[1]
          };
          if (p.hitCP === 1) {
            p.valid = Utils.checkBezierLimits(j1, j2, cp, p.cp2, 2);
            if (p.valid) {
              p.cp1 = cp;
            }
          } else {
            p.valid = Utils.checkBezierLimits(j1, j2, p.cp1, cp, 1);
            if (p.valid) {
              p.cp2 = cp;
            }
          }
          if (p.valid) {
            viz.style('cursor', 'move');
          } else {
            viz.style('cursor', 'not-allowed');
          }
          drawBezierCP(p, p.hitCP);
        }
      }

      // draggable junctions
      if (dg) {
        if (!e.altKey) {
          let agvPt = Utils.snapToLivaAgv(this.p1);
          if (agvPt) {
            this._p1 = agvPt;
          } else {
            let junction = Utils.snapToJunction(this.p1);
            if (junction) {
              this._p1 = [junction.x, junction.y];
            }
          }
        }
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
      if (!e.altKey) {
        this._p1 = Utils.snap(this.p1);
      } else {
        this._p1 = viz.zoom.pixelToCoord(this.p1);
      }
      if (cln) {
        if (!e.altKey) {
          let agvPt = Utils.snapToLivaAgv(this.p1);
          if (agvPt) {
            this._p1 = agvPt;
          } else {
            let junction = Utils.snapToJunction(this.p1);
            if (junction) {
              this._p1 = [junction.x, junction.y];
            }
          }
        }
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

  function trackended(e) {
    /* jshint validthis: true */
    tracked.call(this, e);
    if (scene.mode >= EditMode.CONNECTOR_STRAIGHT && scene.mode < EditMode.LIVE_SET_POSE) {
      if (j1 && j2 && p && p.distance > 0.599) {
        let idx = viz.models.addPath(p, j1, j2);
        scene.activeObject.set(viz.models.rawPaths()[idx]);
      }
      j1 = j2 = p = null;
      drawConnector();
      sendEditMode(EditMode.POINTER);
    } else if (scene.mode === EditMode.LANDMARK) {
      if (lm && lm.valid) {
        if (lm.ob) {
          viz.models.push('dragLandmark');
          lm.ob.x = lm.x;
          lm.ob.y = lm.y;
          viz.models.updateLandmark(lm.ob);
        } else {
          let idx = viz.models.addLandmark(lm);
          scene.activeObject.set(viz.models.rawLandmarks()[idx]);
        }
      }
      lm = null;
      drawLandmark();
      sendEditMode(EditMode.LANDMARK);
    } else if (scene.mode === EditMode.FORBIDDEN_ZONE) {
      if (fz && fz.valid) {
        let idx = viz.models.addForbiddenZone(fz);
        scene.activeObject.set(viz.models.rawForbiddenZones()[idx]);
      }
      fz = null;
      drawForbiddenZone();
      sendEditMode(EditMode.FORBIDDEN_ZONE);
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
      // draggable control points for bezier curve
      if (p && p.hitCP) {
        if (p.valid) {
          viz.models.push('dragBezierCP');
          if (p.hitCP === 1) {
            p.ob.cp1 = p.cp1;
          } else {
            p.ob.cp2 = p.cp2;
          }
          scene.activeObject.setPathDistance(Utils.computeBezierPathLength(p, j1, j2));
        }
        drawBezierCP(p.ob);
        j1 = j2 = p = null;
        sendEditMode(EditMode.POINTER);
      }

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

    var layer = viz.models.rawActiveLayer();

    construct.initial.attrs({
      x: j1.x,
      y: j1.y,
      display: 'block',
      class: `junction active layer-${layer}`
    });

    if (!j2) {
      construct.final.attr('display', 'none');
      construct.connector.attr('display', 'none');
      return;
    }

    construct.final.attrs({
      x: j2.x,
      y: j2.y,
      display: 'block',
      class: `junction active layer-${layer}`
    });

    if (!p) {
      construct.connector.attr('display', 'none');
      return;
    }

    construct.connector.attrs({
      d: Utils.generateConnectorPath(p, j1, j2),
      display: 'block',
      class: `path active layer-${layer}`
    });
  }

  function drawStation(st, j1) {
    if (!st || !j1) {
      construct.station.attr('display', 'none');
      return;
    }

    var layer = viz.models.rawActiveLayer();

    construct.station.attrs({
      transform: `translate(${j1.x},${j1.y})rotate(${st.heading})`,
      'xlink:href': st.heading === Station.Heading.NA ? '#station-headless' : '#station',
      display: 'block',
      class: `station active layer-${layer}`
    });
  }

  function drawDraggables(dg) {
    if (!dg) {
      dg = {};
    }

    var vizJ = construct.drag
      .selectAll('use')
      .data(dg.j || [])
      .join('use');
    vizJ.attrs({
      'xlink:href': '#junction',
      class: (d) => `junction active layer-${d.layer}`,
      x: (d) => d.x,
      y: (d) => d.y
    });

    var p = (dg.p || []).concat(dg.pp || []);
    var vizP = construct.connectors.selectAll('path.path').data(p).join('path');
    vizP
      .attrs({
        class: (pp) => `path active layer-${pp[2].layer}`,
        d: (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1])
      })
      .classed('path-dynamic', (pp) => pp[2].dynamic);

    // overlay rail pattern if the path is tracked
    var vizR = construct.connectors
      .selectAll('path.path-rail')
      .data(_.filter(p, '2.tracked'))
      .join('path');
    vizR.attrs({
      class: (pp) => `path-rail active layer-${pp[2].layer}`,
      d: (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1], true)
    });

    var vizSt = construct.stations
      .selectAll('use')
      .data(dg.st || [])
      .join('use');
    vizSt.attrs({
      class: (d) => `station active layer-${d[1].layer}`,
      transform: (d) => `translate(${dg.j[d[0]].x},${dg.j[d[0]].y})rotate(${d[1].heading})`,
      'xlink:href': (d) =>
        d[1].heading === Station.Heading.NA ? '#station-headless' : '#station'
    });
  }

  function drawBezierCP(p, hitCP) {
    if (!(p && p instanceof Path && p.shape === Path.Shape.BEZIER)) {
      construct.bezier.attr('display', 'none');
      construct.bezierCP.attr('display', 'none');
      return;
    }

    var j1 = viz.models.rawJunctions()[p.j1];
    var j2 = viz.models.rawJunctions()[p.j2];

    if (hitCP) {
      construct.bezier.attrs({
        d: Utils.generateConnectorPath(p, j1, j2),
        display: 'block'
      });
    } else {
      construct.bezier.attr('display', 'none');
    }

    var cpOverlapped = hitCP && p.cp1.x === p.cp2.x && p.cp1.y === p.cp2.y;

    construct.bezierCP.cp1
      .attrs({
        x: p.cp1.x,
        y: p.cp1.y
      })
      .classed('active', hitCP === 1 || cpOverlapped);

    construct.bezierCP.cp2
      .attrs({
        x: p.cp2.x,
        y: p.cp2.y
      })
      .classed('active', hitCP === 2 || cpOverlapped);

    construct.bezierCP.lines[0]
      .attrs({
        x1: j1.x,
        y1: j1.y,
        x2: p.cp1.x,
        y2: p.cp1.y
      })
      .classed('active', hitCP === 1);

    construct.bezierCP.lines[1]
      .attrs({
        x1: j2.x,
        y1: j2.y,
        x2: p.cp2.x,
        y2: p.cp2.y
      })
      .classed('active', hitCP === 2);

    construct.bezierCP.attr('display', 'block');
  }

  function drawLandmark(lm) {
    if (!lm) {
      construct.landmark.attr('display', 'none');
      return;
    }
    construct.landmark
      .attrs({
        x: lm.x,
        y: lm.y,
        display: 'block'
      })
      .classed('dragged', lm.ob);
  }

  function drawForbiddenZone(fz) {
    if (!fz || !fz.points || !fz.points.length || fz.points.length < 3) {
      construct.forbiddenZone.polygon.attr('display', 'none');
      return;
    }
    construct.forbiddenZone.polygon.attrs({
      points: fz.points,
      display: 'block'
    });
  }

  function drawNoRotateZone(fz) {
    if (!fz || !fz.points || !fz.points.length || fz.points.length < 3) {
      construct.noRotateZone.polygon.attr('display', 'none');
      return;
    }
    construct.noRotateZone.polygon.attrs({
      points: fz.points,
      display: 'block'
    });
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
    construct.selectionArea.polygon.attrs({
      points: area.points,
      display: 'block'
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
      class: 'ruler-label',
      transform: (d) => `translate(${d.x},${d.y})scale(1,-1)rotate(90)rotate(${d.rotation})`
    });
    t.text((d) => (d.distance ? `${+d.distance.toFixed(2)} m` : '0'));
  }

  function drawClone(cln) {
    if (!cln || !cln.valid) {
      cln = {};
    }

    var vizJ = construct.drag
      .selectAll('use')
      .data(cln.j || [])
      .join('use');
    vizJ.attrs({
      class: 'junction active',
      'xlink:href': '#junction',
      x: (d) => d.x,
      y: (d) => d.y
    });

    var vizP = construct.connectors
      .selectAll('path.path')
      .data(cln.p || [])
      .join('path');
    vizP
      .attrs({
        class: 'path active',
        d: (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1])
      })
      .classed('path-dynamic', (pp) => pp[2].dynamic);

    // overlay rail pattern if the path is tracked
    var vizR = construct.connectors
      .selectAll('path.path-rail')
      .data(_.filter(cln.p || [], '2.tracked'))
      .join('path');
    vizR.attrs({
      class: 'path-rail active',
      d: (pp) => Utils.generateConnectorPath(pp[2], pp[0], pp[1], true)
    });

    var vizSt = construct.stations
      .selectAll('use')
      .data(cln.st || [])
      .join('use');
    vizSt.attrs({
      class: 'station active',
      transform: (d) => `translate(${cln.j[d[0]].x},${cln.j[d[0]].y})rotate(${d[1].heading})`,
      'xlink:href': (d) =>
        d[1].heading === Station.Heading.NA ? '#station-headless' : '#station'
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

    var landmark = scene.activeObject.get() instanceof Landmark;
    if (m !== EditMode.LANDMARK) {
      if (landmark) {
        scene.activeObject.set(null);
      }
    }
    var forbiddenZone = scene.activeObject.get() instanceof ForbiddenZone;
    if (m !== EditMode.FORBIDDEN_ZONE) {
      if (forbiddenZone) {
        scene.activeObject.set(null);
      }
    }
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
      if (!landmark && !forbiddenZone && !noRotateZone && !locHintZone) {
        scene.activeObject.set(null);
      }
      scene.hoverObject.set(null);
      drawBezierCP();
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
    forbiddenZone: construct.forbiddenZone,
    noRotateZone: construct.noRotateZone,
    locHintZone: construct.locHintZone,
    setEditMode: setEditMode
  };
}
