/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import EditMode from './edit-mode';
import Junction from '../models/junction';
import Path from '../models/path';
import Station from '../models/station';
import hitTest from '../models/hit-test';

export default function (viz, scene) {
  /* Utilities */

  function makeJunction(pt) {
    /* create a new junction or reuse an existing one if found */
    var j = new Junction({
      x: pt[0],
      y: pt[1]
    });

    var hit = viz.models.hitTestJunction(j);
    if (hit instanceof Junction) {
      return hit;
    } else if (!hit) {
      return j;
    } else {
      return null;
    }
  }

  function makePath(j1, j2, mode, hint, excludeJ = []) {
    /* create a new path without crossing or overlapping existing ones */
    if (!Array.isArray(excludeJ)) {
      excludeJ = [excludeJ];
    }
    if (!j1 || !j2) {
      return null;
    }
    if (j1.x === j2.x && j1.y === j2.y) {
      return null;
    }

    var p;
    mode = mode || EditMode.CONNECTOR_STRAIGHT;
    hint = hint || Path.Direction.NORTH;

    if (mode === EditMode.CONNECTOR_STRAIGHT) {
      return makePathStraight(j1, j2, excludeJ);
    } else if (mode === EditMode.CONNECTOR_RIGHT_ANGLED) {
      /* Try hinted direction first, then try another direction. */
      p = makePathRightAngled(j1, j2, hint, excludeJ);
      if (p) {
        return p;
      }
      return makePathRightAngled(
        j1,
        j2,
        hint === Path.Direction.NORTH ? Path.Direction.EAST : Path.Direction.NORTH,
        excludeJ
      );
    } else if (mode === EditMode.CONNECTOR_S_CURVED) {
      /* Try hinted direction first, then try another direction. */
      p = makePathSCurved(j1, j2, hint, excludeJ);
      if (p) {
        return p;
      }
      return makePathSCurved(
        j1,
        j2,
        hint === Path.Direction.NORTH ? Path.Direction.EAST : Path.Direction.NORTH,
        excludeJ
      );
    }

    return null;
  }

  function makePathStraight(j1, j2, excludeJ = []) {
    var p = new Path({
      shape: Path.Shape.STRAIGHT,
      distance: Math.abs(j1.x - j2.x) + Math.abs(j1.y - j2.y)
    });

    if (j1.x === j2.x) {
      if (j1.y > j2.y) {
        p.direction = Path.Direction.NORTH;
      } else {
        p.direction = Path.Direction.SOUTH;
      }
    } else if (j1.y === j2.y) {
      if (j1.x < j2.x) {
        p.direction = Path.Direction.EAST;
      } else {
        p.direction = Path.Direction.WEST;
      }
    } else {
      p = null;
    }

    if (p && viz.models.hitTestPath(j1, j2, p, excludeJ)) {
      p = null;
    }
    return p;
  }

  function makePathRightAngled(j1, j2, hint, excludeJ = []) {
    var p = new Path({
      distance: Math.abs(j1.x - j2.x) + Math.abs(j1.y - j2.y)
    });

    if (j1.x === j2.x || j1.y === j2.y) {
      return null;
    }

    if (hint === Path.Direction.NORTH) {
      if (j1.y > j2.y) {
        p.direction = Path.Direction.NORTH;
        p.shape = j1.x > j2.x ? Path.Shape.BEND_LEFT : Path.Shape.BEND_RIGHT;
      } else {
        p.direction = Path.Direction.SOUTH;
        p.shape = j1.x > j2.x ? Path.Shape.BEND_RIGHT : Path.Shape.BEND_LEFT;
      }
    } else if (hint === Path.Direction.EAST) {
      if (j1.x < j2.x) {
        p.direction = Path.Direction.EAST;
        p.shape = j1.y < j2.y ? Path.Shape.BEND_RIGHT : Path.Shape.BEND_LEFT;
      } else {
        p.direction = Path.Direction.WEST;
        p.shape = j1.y < j2.y ? Path.Shape.BEND_LEFT : Path.Shape.BEND_RIGHT;
      }
    } else {
      p = null;
    }

    if (p && viz.models.hitTestPath(j1, j2, p, excludeJ)) {
      p = null;
    }
    return p;
  }

  function makePathSCurved(j1, j2, hint, excludeJ = []) {
    var p = new Path({
      shape: Path.Shape.S_CURVE,
      distance: Math.abs(j1.x - j2.x) + Math.abs(j1.y - j2.y)
    });

    if (j1.x === j2.x || j1.y === j2.y) {
      return null;
    }

    if (hint === Path.Direction.NORTH) {
      if (j1.y > j2.y) {
        p.direction = Path.Direction.NORTH;
      } else {
        p.direction = Path.Direction.SOUTH;
      }
    } else if (hint === Path.Direction.EAST) {
      if (j1.x < j2.x) {
        p.direction = Path.Direction.EAST;
      } else {
        p.direction = Path.Direction.WEST;
      }
    } else {
      p = null;
    }

    if (p && viz.models.hitTestPath(j1, j2, p, excludeJ)) {
      p = null;
    }
    return p;
  }

  function makeStation(j1) {
    var stations = viz.models.lookupStationsFromJunction(j1);
    if (Array.isArray(stations) && stations.length <= Station.Direction.NA) {
      var i;
      for (i = 0; i < stations.length; i++) {
        if (i !== stations[i].direction) {
          break;
        }
      }
      return new Station({
        direction: i
      });
    }
    return null;
  }

  function makeDraggables(pt, dg) {
    /**
     * Updates draggables (dg) and returns the validity of the dragged position
     *
     * pt: new drag position
     * dg: {
     *   ctx:  original selected objects
     *   cp:   original junction as control point
     *   obs:   original junctions
     *   obsPP: original connections [[j1, j2, p]]
     *   obsP:   original paths [[j1, j2, p]]
     *   obsSt:   original stations for each junctions [[st]]
     *   st:   original stations with obs junction idx [[j, st]]
     *   off: original junctions offset from control point
     *   j:    new junctions (dragged)
     *   p:   new paths (dragged) [[j1, j2, p]]
     *   pp:   new connections (dragged) [[j1, j2, p]]
     *   collapsedP:   original paths that are collapsed due to dragging
     * }
     */
    if (!dg || !dg.obs || !dg.obs.length) {
      return false;
    }
    if (!dg.obsPP) {
      let junctions = viz.models.rawJunctions();

      dg.obsPP = viz.models.lookupConnections(dg.obs);
      dg.obsPP.j = dg.obsPP.map((pp) => (dg.obs.indexOf(pp[0]) < 0 ? pp[0] : pp[1]));
      dg.off = dg.obs.map((ob) => ({
        x: ob.x - dg.cp.x,
        y: ob.y - dg.cp.y
      }));
      dg.obsP = viz.models
        .lookupPathsFromJunctions(dg.obs)
        .map((p) => [junctions[p.j1], junctions[p.j2], p]);

      dg.obsSt = dg.obs.map((ob) => viz.models.lookupStationsFromJunction(ob));
      dg.st = [];
      for (let idx = 0; idx < dg.obs.length; ++idx) {
        for (let st of dg.obsSt[idx]) {
          dg.st.push([idx, st]);
        }
      }
    }

    // create dragged junction
    dg.j = dg.obs.map(function (ob, i) {
      let _pt = viz.zoom.computeSnap([pt[0] + dg.off[i].x, pt[1] + dg.off[i].y], 0.05);
      return new Junction({
        x: _pt[0],
        y: _pt[1]
      });
    });

    var j_overlap = [];
    for (let idx = 0; idx < dg.obs.length; ++idx) {
      let j = dg.j[idx];
      let ob = dg.obs[idx];
      let hit = viz.models.hitTestJunction(j, dg.obs);

      if (hit instanceof Junction && viz.models.isJunctionMergeAllowed(hit, ob)) {
        dg.j[idx] = hit;
        j_overlap.push([hit, idx]);
      } else if (hit) {
        return false;
      }
    }

    // create dragged path
    dg.p = [];
    dg.p.p = [];
    for (let idx = 0; idx < dg.obsP.length; ++idx) {
      let [j1, j2, p] = dg.obsP[idx];
      let idxj1 = dg.obs.indexOf(j1);
      let idxj2 = dg.obs.indexOf(j2);
      let nj1 = dg.j[idxj1];
      let nj2 = dg.j[idxj2];

      p = Object.assign(new Path(p), {
        ob: p
      });

      // create dragged path
      let mode;
      if (nj1.x === nj2.x || nj1.y === nj2.y) {
        mode = EditMode.CONNECTOR_STRAIGHT;
      } else if (p.shape <= Path.Shape.S_CURVE) {
        mode = EditMode.CONNECTOR_S_CURVED;
      } else {
        mode = EditMode.CONNECTOR_RIGHT_ANGLED;
      }
      let hint = Path.isNS(p) ? Path.Direction.NORTH : Path.Direction.EAST;
      let newP = makePath(nj1, nj2, mode, hint, dg.obs);
      if (!newP) {
        return false;
      }

      p.direction = newP.direction;
      p.shape = newP.shape;
      p.distance = newP.distance;
      p.j1 = idxj1;
      p.j2 = idxj2;

      dg.p.push([nj1, nj2, p]);
      dg.p.p.push(p);
    }

    // create dragged connections
    dg.pp = [];
    dg.pp.p = [];
    dg.collapsedP = [];
    var len = dg.obsPP.length;
    for (let idx = 0; idx < len; ++idx) {
      let [j1, j2, p] = dg.obsPP[idx];
      let fixJ = dg.obsPP.j[idx];
      p = Object.assign(new Path(p), {
        ob: p
      });

      // replace original junction with dragged junction
      if (j1 === fixJ) {
        p.j1 = idx + dg.obs.length;
        p.j2 = dg.obs.indexOf(j2);
        j2 = dg.j[p.j2];
      } else {
        p.j1 = dg.obs.indexOf(j1);
        p.j2 = idx + dg.obs.length;
        j1 = dg.j[p.j1];
      }

      // collapsed path
      if (j1 === j2) {
        dg.collapsedP.push(p.ob);
        continue;
      }

      // create dragged path
      let mode;
      if (j1.x === j2.x || j1.y === j2.y) {
        mode = EditMode.CONNECTOR_STRAIGHT;
      } else if (p.shape <= Path.Shape.S_CURVE) {
        mode = EditMode.CONNECTOR_S_CURVED;
      } else {
        mode = EditMode.CONNECTOR_RIGHT_ANGLED;
      }
      let hint = Path.isNS(p) ? Path.Direction.NORTH : Path.Direction.EAST;
      let newP = makePath(j1, j2, mode, hint, dg.obs);
      if (!newP) {
        return false;
      }
      p.direction = newP.direction;
      p.shape = newP.shape;
      p.distance = newP.distance;

      dg.pp.push([j1, j2, p]);
      dg.pp.p.push(p);
    }

    // hit test for new paths with new paths (plus paths linked to merged junction - for branch test)
    var junctions = dg.j.concat(dg.obsPP.j);
    var paths = dg.p.p.concat(dg.pp.p);

    for (let idx = 0; idx < j_overlap.length; ++idx) {
      let [hit, jid] = j_overlap[idx];
      for (let pp of viz.models.lookupConnections(hit)) {
        let [j1, j2, p] = pp;
        if (dg.collapsedP.indexOf(p) >= 0) {
          continue;
        }
        p = new Path(p);
        paths.push(p);
        if (j1 === hit) {
          p.j1 = jid;
          p.j2 = junctions.length;
          junctions.push(j2);
        } else {
          p.j1 = junctions.length;
          p.j2 = jid;
          junctions.push(j1);
        }
      }
    }

    for (let pp of dg.p.concat(dg.pp)) {
      if (hitTest.path(junctions, paths, pp[0], pp[1], pp[2])) {
        return false;
      }
    }
    return true;
  }

  function saveDraggables(dg) {
    viz.models.push('dragJunction');

    var dgRemoved = dg.collapsedP.slice(0);
    var dgAdded = [];

    for (let i = 0; i < dg.j.length; ++i) {
      let j = dg.j[i];
      let ob = dg.obs[i];
      let idx = viz.models.rawJunctions().indexOf(j);
      if (idx >= 0) {
        // junction merged
        for (let st of dg.obsSt[i]) {
          st.j = idx;
        }
        j.rfid = j.rfid || ob.rfid;
        dgRemoved.push(ob);
        dgAdded.push(j);
      } else {
        ob.x = j.x;
        ob.y = j.y;
      }
    }

    var paths = (dg.p || []).concat(dg.pp || []);
    for (let pp of paths) {
      let [j1, j2, p] = pp;
      let idxj1 = viz.models.rawJunctions().indexOf(j1);
      let idxj2 = viz.models.rawJunctions().indexOf(j2);

      if (idxj1 >= 0) {
        p.ob.j1 = idxj1;
      }
      if (idxj2 >= 0) {
        p.ob.j2 = idxj2;
      }
      p.ob.direction = p.direction;
      p.ob.shape = p.shape;
      p.ob.distance = p.distance;
      viz.models.updateBranch(p.ob);
    }

    // remove collapsed path then remove collapsed junction
    for (let ob of dgRemoved) {
      if (ob instanceof Junction) {
        viz.models.removeJunction(ob);
      } else if (ob instanceof Path) {
        viz.models.removePath(ob);
      }
    }

    viz.models.triggerDirty();

    return dg.ctx.filter((ob) => dgRemoved.indexOf(ob) < 0).concat(dgAdded);
  }

  function makeClone(pt, cln) {
    /**
     * Updates clone (cln) and returns the validity of the dragged position
     *
     * pt: new drag position
     * dg: {
     *   ctx:  original selected objects
     *   cp:   original junction as control point
     *   obs:   original junctions
     *   floating:  list of floating flag for respective obs.
     *   obsP:   original paths [[j1, j2, p]]
     *   obsSt:   original stations for each junctions [[st]]
     *   st:   original stations with obs junction idx [[j, st]]
     *   off: original junctions offset from control point
     *   j:    new junctions (dragged)
     *   p:   new paths (dragged) [[j1, j2, p]]
     * }
     */
    if (!cln || !cln.obs || !cln.obs.length) {
      return false;
    }

    if (!cln.off) {
      cln.off = cln.obs.map((ob) => ({
        x: ob.x - cln.cp.x,
        y: ob.y - cln.cp.y
      }));

      let junctions = viz.models.rawJunctions();
      cln.floating = cln.obs.map(function (ob) {
        let idx = junctions.indexOf(ob);
        for (let c of cln.ctx) {
          if (c instanceof Path && (c.j1 === idx || c.j2 === idx)) {
            return false;
          }
        }
        return true;
      });

      cln.obsP = cln.ctx
        .filter((ob) => ob instanceof Path)
        .map((p) => [junctions[p.j1], junctions[p.j2], p]);

      cln.st = cln.ctx
        .filter((ob) => ob instanceof Station)
        .map(function (st) {
          let j = junctions[st.j];
          return [cln.obs.indexOf(j), st];
        });
    }

    // create junction
    cln.j = cln.obs.map(function (ob, i) {
      let _pt = viz.zoom.computeSnap([pt[0] + cln.off[i].x, pt[1] + cln.off[i].y], 0.05);
      let j = new Junction(ob);
      j.x = _pt[0];
      j.y = _pt[1];
      return j;
    });

    var j_overlap = [];
    for (let idx = 0; idx < cln.obs.length; ++idx) {
      let j = cln.j[idx];
      let ob = cln.obs[idx];
      let hit = viz.models.hitTestJunction(j);

      if (
        hit instanceof Junction &&
        isCloneJunctionMergeAllowed(
          viz.models.rawJunctions(),
          viz.models.rawStations(),
          hit,
          ob
        )
      ) {
        cln.j[idx] = hit;
        j_overlap.push([hit, idx]);
      } else if (hit) {
        return false;
      } else if (!hit && cln.floating[idx]) {
        return false;
      }
    }

    // create path
    cln.p = [];
    cln.p.p = [];
    for (let idx = 0; idx < cln.obsP.length; ++idx) {
      let [j1, j2, p] = cln.obsP[idx];
      let idxj1 = cln.obs.indexOf(j1);
      let idxj2 = cln.obs.indexOf(j2);
      let nj1 = cln.j[idxj1];
      let nj2 = cln.j[idxj2];

      p = Object.assign(new Path(p), {
        ob: p,
        bj1: undefined,
        bj2: undefined
      });
      let mode;
      if (nj1.x === nj2.x || nj1.y === nj2.y) {
        mode = EditMode.CONNECTOR_STRAIGHT;
      } else if (p.shape <= Path.Shape.S_CURVE) {
        mode = EditMode.CONNECTOR_S_CURVED;
      } else {
        mode = EditMode.CONNECTOR_RIGHT_ANGLED;
      }
      let hint = Path.isNS(p) ? Path.Direction.NORTH : Path.Direction.EAST;
      let newP = makePath(nj1, nj2, mode, hint);
      if (!newP) {
        return false;
      }

      p.direction = newP.direction;
      p.shape = newP.shape;
      p.distance = newP.distance;
      p.j1 = idxj1;
      p.j2 = idxj2;

      cln.p.push([nj1, nj2, p]);
      cln.p.p.push(p);
    }

    // Hit test path and path for branching
    var junctions = cln.j.slice(0);
    var paths = cln.p.p.slice(0);

    for (let idx = 0; idx < j_overlap.length; ++idx) {
      let [hit, jid] = j_overlap[idx];
      for (let pp of viz.models.lookupConnections(hit)) {
        let [j1, j2, p] = pp;
        p = new Path(p);
        paths.push(p);
        if (j1 === hit) {
          p.j1 = jid;
          p.j2 = junctions.length;
          junctions.push(j2);
        } else {
          p.j1 = junctions.length;
          p.j2 = jid;
          junctions.push(j1);
        }
      }
    }

    for (let pp of cln.p) {
      if (hitTest.path(junctions, paths, pp[0], pp[1], pp[2])) {
        return false;
      }
    }

    return true;
  }

  function saveClone(cln) {
    viz.models.push('cloneObjects');

    var clonePaths = [];
    var cloneJunctions = [];
    var cloneStation = [];

    for (let pp of cln.p) {
      let [j1, j2, p] = pp;
      j1 = cln.j[p.j1];
      j2 = cln.j[p.j2];

      let idx = viz.models.addPath(p, j1, j2, false);
      let path = viz.models.rawPaths()[idx];

      clonePaths.push(path);
      let cj1 = viz.models.rawJunctions()[path.j1];
      let cj2 = viz.models.rawJunctions()[path.j2];
      if (cloneJunctions.indexOf(cj1) < 0) {
        cloneJunctions.push(cj1);
      }
      if (cloneJunctions.indexOf(cj2) < 0) {
        cloneJunctions.push(cj2);
      }

      if (viz.models.rawJunctions().indexOf(j1) < 0) {
        // update j from models
        cln.j[cln.j.indexOf(j1)] = cj1;
      }
      if (viz.models.rawJunctions().indexOf(j2) < 0) {
        // update j from models
        cln.j[cln.j.indexOf(j2)] = cj2;
      }
    }

    for (let [idx, st] of cln.st) {
      let i = viz.models.addStation(new Station(st), cln.j[idx], false);
      cloneStation.push(viz.models.rawStations()[i]);
    }

    viz.models.triggerDirty();

    return cloneJunctions.concat(clonePaths).concat(cloneStation);
  }

  function makeArea(pt1, pt2) {
    if (!pt2) {
      return {
        points: [pt1, pt1, pt1, pt1]
      };
    }
    // generate points of a rectangle in anti-clockwise order
    var dx = pt2[0] - pt1[0];
    var dy = pt2[1] - pt1[1];
    var dxdy = dx * dy;

    if (dxdy === 0) {
      return false;
    } else if (dxdy > 0) {
      return {
        points: [pt1, [pt2[0], pt1[1]], pt2, [pt1[0], pt2[1]]]
      };
    } else {
      return {
        points: [pt1, [pt1[0], pt2[1]], pt2, [pt2[0], pt1[1]]]
      };
    }
  }

  function objectsInArea(pt1, pt2) {
    return viz.models.hitTestRect(pt1, pt2);
  }

  function snap(pt, snapDistance) {
    return viz.zoom.computeSnap(viz.zoom.pixelToCoord(pt), snapDistance);
  }

  function snapToObject(pt) {
    /* snap to an existing junction, path, or station. */
    var sd = viz.zoom.getSnapDistance();
    var lsd = viz.zoom.getLowerSnapDistance();
    var hit;

    if (lsd) {
      hit = viz.models.hitTestPoint(snap(pt, lsd));
    }
    if (!hit) {
      hit = viz.models.hitTestPoint(snap(pt, sd));
    }
    return hit ? hit : null;
  }

  function snapToJunction(pt) {
    /* snap to an existing junction. */
    var j = snapToObject(pt);
    return j instanceof Junction ? j : null;
  }

  function withinCorners(pt) {
    var corners = viz.zoom.getCorners();
    return (
      pt[0] >= corners.left &&
      pt[0] <= corners.right &&
      pt[1] >= corners.top &&
      pt[1] <= corners.bottom
    );
  }

  function limitMovement(initial_pt, final_pt) {
    if (scene.mode === EditMode.CONNECTOR_STRAIGHT) {
      var diff0 = final_pt[0] - initial_pt[0];
      var diff1 = final_pt[1] - initial_pt[1];
      if (Math.abs(diff0) < Math.abs(diff1)) {
        return [initial_pt[0], final_pt[1]];
      } else {
        return [final_pt[0], initial_pt[1]];
      }
    }
    return final_pt;
  }

  return {
    makeJunction: makeJunction,
    makePath: makePath,
    makeStation: makeStation,
    makeDraggables: makeDraggables,
    saveDraggables: saveDraggables,
    makeClone: makeClone,
    saveClone: saveClone,
    makeArea: makeArea,
    objectsInArea: objectsInArea,
    snap: snap,
    snapToObject: snapToObject,
    snapToJunction: snapToJunction,
    withinCorners: withinCorners,
    limitMovement: limitMovement,
    generateConnectorPath: generateConnectorPath,
    generateRulerPath: generateRulerPath
  };
}

/* SVG helper function */
function generateConnectorPath(path, j1, j2, trunkOnly) {
  var [flow, facing] = trunkOnly ? [0, 0] : [path.flow, path.facing];
  if (flow && facing >= Path.Facing.FORWARD_BI) {
    facing -= 2;
  }

  var radius;
  var arcP0, arcP1, sweep;
  var bezierP0, bezierP1, bezierC0, bezierC1;
  var extra = '';

  if (path.shape === Path.Shape.STRAIGHT) {
    if (flow || facing) {
      extra = ' ' + generateArrow([j1.x, j1.y], [j2.x, j2.y], path.direction, flow, facing);
    }
    return `M ${j1.x},${j1.y} L ${j2.x},${j2.y}${extra}`;
  } else if (path.shape === Path.Shape.BEND_LEFT || path.shape === Path.Shape.BEND_RIGHT) {
    radius = Math.min(Math.abs(j1.x - j2.x), Math.abs(j1.y - j2.y)) / 2.0;
    radius = Math.min(radius, 1.0);

    switch (path.direction) {
      case Path.Direction.NORTH:
        arcP0 = [j1.x, j2.y + radius];
        arcP1 = [j1.x + (path.shape === Path.Shape.BEND_LEFT ? -radius : radius), j2.y];
        sweep = path.shape === Path.Shape.BEND_LEFT ? 0 : 1;
        break;
      case Path.Direction.SOUTH:
        arcP0 = [j1.x, j2.y - radius];
        arcP1 = [j1.x - (path.shape === Path.Shape.BEND_LEFT ? -radius : radius), j2.y];
        sweep = path.shape === Path.Shape.BEND_LEFT ? 0 : 1;
        break;
      case Path.Direction.EAST:
        arcP0 = [j2.x - radius, j1.y];
        arcP1 = [j2.x, j1.y + (path.shape === Path.Shape.BEND_LEFT ? -radius : radius)];
        sweep = path.shape === Path.Shape.BEND_LEFT ? 0 : 1;
        break;
      case Path.Direction.WEST:
        arcP0 = [j2.x + radius, j1.y];
        arcP1 = [j2.x, j1.y - (path.shape === Path.Shape.BEND_LEFT ? -radius : radius)];
        sweep = path.shape === Path.Shape.BEND_LEFT ? 0 : 1;
        break;
    }

    if (flow || facing) {
      if (Path.isNS(path)) {
        extra = ' ' + generateArrow([j1.x, j1.y], [j1.x, j2.y], path.direction, flow, facing);
        extra +=
          ' ' +
          generateArrow(
            [j2.x, j2.y],
            [j1.x, j2.y],
            (path.direction + (path.shape === Path.Shape.BEND_LEFT ? 3 : 1)) % 4,
            flow,
            facing
          );
      } else if (Path.isEW(path)) {
        extra = ' ' + generateArrow([j1.x, j1.y], [j2.x, j1.y], path.direction, flow, facing);
        extra +=
          ' ' +
          generateArrow(
            [j2.x, j2.y],
            [j2.x, j1.y],
            (path.direction + (path.shape === Path.Shape.BEND_LEFT ? 3 : 1)) % 4,
            flow,
            facing
          );
      }
    }

    return `M ${j1.x},${j1.y} L ${arcP0} A ${radius} ${radius}, 0, 0, ${sweep}, ${arcP1} L ${j2.x},${j2.y}${extra}`;
  } else if (path.shape === Path.Shape.S_CURVE) {
    radius = Math.min(Math.abs(j1.x - j2.x), Math.abs(j1.y - j2.y)) / 4.0;

    switch (path.direction) {
      case Path.Direction.NORTH:
        bezierP0 = [j1.x, (j1.y + j2.y) / 2 + radius * 1.0];
        bezierC0 = [j1.x, (j1.y + j2.y) / 2 + radius * 0.5];
        bezierP1 = [j2.x, (j1.y + j2.y) / 2 - radius * 1.0];
        bezierC1 = [j2.x, (j1.y + j2.y) / 2 - radius * 0.5];
        break;
      case Path.Direction.SOUTH:
        bezierP0 = [j1.x, (j1.y + j2.y) / 2 - radius * 1.0];
        bezierC0 = [j1.x, (j1.y + j2.y) / 2 - radius * 0.5];
        bezierP1 = [j2.x, (j1.y + j2.y) / 2 + radius * 1.0];
        bezierC1 = [j2.x, (j1.y + j2.y) / 2 + radius * 0.5];
        break;
      case Path.Direction.EAST:
        bezierP0 = [(j1.x + j2.x) / 2 - radius * 1.0, j1.y];
        bezierC0 = [(j1.x + j2.x) / 2 - radius * 0.5, j1.y];
        bezierP1 = [(j1.x + j2.x) / 2 + radius * 1.0, j2.y];
        bezierC1 = [(j1.x + j2.x) / 2 + radius * 0.5, j2.y];
        break;
      case Path.Direction.WEST:
        bezierP0 = [(j1.x + j2.x) / 2 + radius * 1.0, j1.y];
        bezierC0 = [(j1.x + j2.x) / 2 + radius * 0.5, j1.y];
        bezierP1 = [(j1.x + j2.x) / 2 - radius * 1.0, j2.y];
        bezierC1 = [(j1.x + j2.x) / 2 - radius * 0.5, j2.y];
        break;
    }

    if (flow || facing) {
      if (Path.isNS(path)) {
        extra =
          ' ' +
          generateArrow([j1.x, j1.y], [j1.x, (j1.y + j2.y) / 2], path.direction, flow, facing);
        extra +=
          ' ' +
          generateArrow([j2.x, j2.y], [j2.x, (j1.y + j2.y) / 2], path.direction, flow, facing);
      } else if (Path.isEW(path)) {
        extra =
          ' ' +
          generateArrow([j1.x, j1.y], [(j1.x + j2.x) / 2, j1.y], path.direction, flow, facing);
        extra +=
          ' ' +
          generateArrow([j2.x, j2.y], [(j1.x + j2.x) / 2, j2.y], path.direction, flow, facing);
      }
    }

    return `M ${j1.x},${j1.y} L ${bezierP0} C ${bezierC0},${bezierC1},${bezierP1} L ${j2.x},${j2.y}${extra}`;
  }
}

function generateArrow(pt1, pt2, direction, flow, facing) {
  // compute half height and half width for flow arrow
  var h =
    Math.max(
      0.1,
      Math.min(0.2, Math.max(Math.abs(pt1[0] - pt2[0]), Math.abs(pt1[1] - pt2[1])) / 4.0)
    ) / 2.0;
  var w = (h * 8.0) / 5.0;

  // compute half height and half width for facing arrow
  var h2 = h / 0.75;
  var w2 = (h2 * 4.0) / 5.0;

  var midPt = [(pt1[0] + pt2[0]) / 2, (pt1[1] + pt2[1]) / 2];
  var offH = facing ? h2 : 0;
  var headingRad = [-Math.PI / 2, 0, Math.PI / 2, Math.PI][direction];
  var cosH = Math.cos(headingRad);
  var sinH = Math.sin(headingRad);

  var extra = '';
  if (flow) {
    extra +=
      ' M' +
      [
        [-h, w],
        [h, 0],
        [-h, -w]
      ]
        .map(rp)
        .join('L');
  }

  var pts;
  switch (facing) {
    case Path.Facing.FORWARD_UNI:
      pts = [
        [0, w2],
        [h2, 0],
        [0, -w2],
        [-h2, -w2],
        [0, 0],
        [-h2, w2]
      ];
      break;
    case Path.Facing.REVERSE_UNI:
      pts = [
        [h2, w2],
        [0, 0],
        [h2, -w2],
        [0, -w2],
        [-h2, 0],
        [0, w2]
      ];
      break;
    case Path.Facing.FORWARD_BI:
      pts = [
        [0, w2],
        [h2, 0],
        [0, -w2],
        [-h2, 0]
      ];
      break;
    case Path.Facing.REVERSE_BI:
      h2 *= 0.88;
      pts = [
        [h2, w2],
        [-h2, -w2],
        [h2, -w2],
        [-h2, w2]
      ];
      break;
  }
  if (pts) {
    offH = flow ? (facing === Path.Facing.REVERSE_UNI ? -h - h2 / 2 : -h2 / 2) : 0;
    pts = pts.map(rp);
    pts.push(pts[0]);
    pts.push(pts[1]); // repeat to fill corner
    extra += ' M' + pts.join('L');
  }

  return extra;

  function rp(pt) {
    // rotate point
    return [
      midPt[0] + cosH * (pt[0] + offH) - sinH * pt[1],
      midPt[1] + sinH * (pt[0] + offH) + cosH * pt[1]
    ];
  }
}

function generateRulerPath(r) {
  if (!Array.isArray(r) || r.length < 2) {
    return ['', '', '', []];
  }

  const offset2 = 0.05;
  const offset3 = 0.025;
  const offsetLabel = 0.2;

  var j1;
  var j2 = r[0];
  var v, u, mag, rot;

  var d = `M ${j2.x},${j2.y}`;
  var d2 = '';
  var d3 = '';

  var nextLabel = 0;
  var dist = 0;
  var labels = [];

  for (let i = 1; i < r.length; ++i) {
    j1 = j2;
    j2 = r[i];
    d += ` L ${j2.x},${j2.y}`;

    v = [j2.x - j1.x, j2.y - j1.y];
    mag = Math.sqrt(v[0] * v[0] + v[1] * v[1]);
    v = [v[0] / mag, v[1] / mag];
    u = [v[1], -v[0]];
    rot = (Math.atan2(v[1], v[0]) * 180) / Math.PI;
    if (rot > 90) {
      rot -= 180;
    } else if (rot < -90) {
      rot += 180;
    }

    d2 += ` M ${j1.x + u[0] * offset2},${j1.y + u[1] * offset2} L ${j2.x + u[0] * offset2},${
      j2.y + u[1] * offset2
    }`;
    d3 += ` M ${j1.x + u[0] * offset3},${j1.y + u[1] * offset3} L ${j2.x + u[0] * offset3},${
      j2.y + u[1] * offset3
    }`;

    let curDist = 0;
    while (mag > 0) {
      let diff = Math.min(nextLabel - dist, mag);
      mag -= diff;
      dist += diff;
      curDist += diff;
      if (dist >= nextLabel) {
        labels.push({
          x: j1.x + v[0] * curDist + u[0] * offsetLabel,
          y: j1.y + v[1] * curDist + u[1] * offsetLabel,
          rotation: rot,
          distance: dist
        });
        nextLabel += 2;
      }
    }
  }

  if (nextLabel - dist > 1.2) {
    labels.pop();
  }
  labels.push({
    x: j2.x + u[0] * offsetLabel,
    y: j2.y + u[1] * offsetLabel,
    rotation: rot,
    distance: dist
  });

  return [d, d2, d3, labels];
}

function isCloneJunctionMergeAllowed(junctions, stations, junction1, junction2) {
  var idx1 = junctions.indexOf(junction1);
  var idx2 = junctions.indexOf(junction2);
  if (idx1 < 0 || idx2 < 0) {
    return false;
  }
  var indexes = [idx1, idx2];
  var stationsDirection = {};
  for (let station of stations) {
    // check if station overlap between j1 and j2
    if (indexes.includes(station.j)) {
      if (stationsDirection[station.direction]) {
        return false;
      }
      stationsDirection[station.direction] = true;
    }
  }
  return true;
}
