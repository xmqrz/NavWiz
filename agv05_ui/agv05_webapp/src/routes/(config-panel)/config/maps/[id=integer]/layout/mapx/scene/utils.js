/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import EditMode from './edit-mode';
import Junction from '../models/junction';
import Landmark from '../models/landmark';
import Path from '../models/path';
import Station from '../models/station';
import hitTest from '../models/hit-test';

const AGV_SNAP_DISTANCE = 0.2;
const BEZIER_CP_RADIUS = 0.05;

export default function (viz, _scene) {
  /* Utilities */

  function makeJunction(pt, isSnap = true) {
    /* create a new junction or reuse an existing one if found */
    var hit = snapToJunction(pt);
    if (hit) {
      return hit;
    }

    if (isSnap) {
      let agvPt = snapToLivaAgv(pt);
      pt = agvPt || snap(pt);
    } else {
      pt = viz.zoom.pixelToCoord(pt);
    }
    var j = new Junction({
      x: pt[0],
      y: pt[1]
    });

    hit = viz.models.hitTestJunction(j);
    if (hit instanceof Junction) {
      return hit;
    } else if (!hit) {
      return j;
    } else {
      return null;
    }
  }

  function makePath(j1, j2, mode) {
    /* create a new path without crossing or overlapping existing ones */
    if (!j1 || !j2) {
      return null;
    }
    if (j1.x === j2.x && j1.y === j2.y) {
      return null;
    }

    mode = mode || EditMode.CONNECTOR_STRAIGHT;

    if (mode === EditMode.CONNECTOR_STRAIGHT) {
      return makePathStraight(j1, j2);
    } else if (mode === EditMode.CONNECTOR_BEZIER) {
      return makePathBezier(j1, j2);
    }

    return null;
  }

  function makePathStraight(j1, j2, excludeJ) {
    var dx = j1.x - j2.x;
    var dy = j1.y - j2.y;
    var p = new Path({
      shape: Path.Shape.STRAIGHT,
      distance: Math.sqrt(dx * dx + dy * dy)
    });

    if (p && viz.models.hitTestPath(j1, j2, p, excludeJ)) {
      p = null;
    }
    return p;
  }

  function makePathBezier(j1, j2, cp1, cp2, excludeJ) {
    var dx = j2.x - j1.x;
    var dy = j2.y - j1.y;
    cp1 = cp1 || {
      x: j1.x - dy / 4,
      y: j1.y + dx / 4
    };
    cp2 = cp2 || {
      x: j2.x + dy / 4,
      y: j2.y - dx / 4
    };
    var p = new Path({
      cp1: cp1,
      cp2: cp2,
      shape: Path.Shape.BEZIER
    });
    p.distance = computeBezierPathLength(p, j1, j2);

    if (p && viz.models.hitTestPath(j1, j2, p, excludeJ)) {
      p = null;
    }
    return p;
  }

  function makeStation(j1) {
    var stations = viz.models.lookupStationsFromJunction(j1);
    if (Array.isArray(stations) && stations.length <= Station.Heading.SEGMENTS) {
      var i;
      for (i = 0; i < stations.length; i++) {
        if (i * Station.Heading.RESOLUTION !== stations[i].heading) {
          break;
        }
      }
      return new Station({
        heading: i * Station.Heading.RESOLUTION
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
      let _pt = [pt[0] + dg.off[i].x, pt[1] + dg.off[i].y];
      return new Junction({
        x: _pt[0],
        y: _pt[1],
        layer: ob.layer
      });
    });

    for (let idx = 0; idx < dg.obs.length; ++idx) {
      let j = dg.j[idx];
      let ob = dg.obs[idx];
      let hit = viz.models.hitTestJunction(j, dg.obs);

      if (hit instanceof Junction && viz.models.isJunctionMergeAllowed(hit, ob)) {
        dg.j[idx] = hit;
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
      if (p.shape === Path.Shape.BEZIER) {
        p.cp1 = {
          x: p.ob.cp1.x - j1.x + nj1.x,
          y: p.ob.cp1.y - j1.y + nj1.y
        };
        p.cp2 = {
          x: p.ob.cp2.x - j2.x + nj2.x,
          y: p.ob.cp2.y - j2.y + nj2.y
        };
      }

      let newP;
      if (p.shape === Path.Shape.STRAIGHT) {
        newP = makePathStraight(nj1, nj2, dg.obs);
      } else if (p.shape === Path.Shape.BEZIER) {
        newP = makePathBezier(nj1, nj2, p.cp1, p.cp2, dg.obs);
      }
      if (!newP) {
        return false;
      }

      p.cp1 = newP.cp1;
      p.cp2 = newP.cp2;
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

      if (p.shape === Path.Shape.BEZIER) {
        p.cp1 = {
          x: p.ob.cp1.x - j1.x,
          y: p.ob.cp1.y - j1.y
        };
        p.cp2 = {
          x: p.ob.cp2.x - j2.x,
          y: p.ob.cp2.y - j2.y
        };
      }

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
      // move CP1 & CP2 according to original offset
      if (p.shape === Path.Shape.BEZIER) {
        p.cp1.x += j1.x;
        p.cp1.y += j1.y;
        p.cp2.x += j2.x;
        p.cp2.y += j2.y;
      }

      // collapsed path
      if (j1 === j2) {
        dg.collapsedP.push(p.ob);
        continue;
      }

      // create dragged path
      let newP;
      if (p.shape === Path.Shape.STRAIGHT) {
        newP = makePathStraight(j1, j2, dg.obs);
      } else if (p.shape === Path.Shape.BEZIER) {
        newP = makePathBezier(j1, j2, p.cp1, p.cp2, dg.obs);
      }
      if (!newP) {
        return false;
      }
      p.cp1 = newP.cp1;
      p.cp2 = newP.cp2;
      p.distance = newP.distance;

      dg.pp.push([j1, j2, p]);
      dg.pp.p.push(p);
    }

    // hit test for new paths with new paths
    var junctions = dg.j.concat(dg.obsPP.j);
    var paths = dg.p.p.concat(dg.pp.p);
    for (let p of dg.p.concat(dg.pp)) {
      if (hitTest.path(junctions, paths, p[0], p[1], p[2], viz.zoom.getSnapDistance() / 2)) {
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
      p.ob.cp1 = p.cp1;
      p.ob.cp2 = p.cp2;
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
      let _pt = [pt[0] + cln.off[i].x, pt[1] + cln.off[i].y];
      let j = new Junction(ob);
      j.x = _pt[0];
      j.y = _pt[1];
      return j;
    });

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
      } else if (hit) {
        return false;
      } else if (!hit && cln.floating[idx]) {
        return false;
      }
    }

    // create path
    cln.p = [];
    for (let idx = 0; idx < cln.obsP.length; ++idx) {
      let [j1, j2, p] = cln.obsP[idx];
      let idxj1 = cln.obs.indexOf(j1);
      let idxj2 = cln.obs.indexOf(j2);
      let nj1 = cln.j[idxj1];
      let nj2 = cln.j[idxj2];

      p = Object.assign(new Path(p), {
        ob: p
      });
      if (p.shape === Path.Shape.BEZIER) {
        p.cp1 = {
          x: p.ob.cp1.x - j1.x + nj1.x,
          y: p.ob.cp1.y - j1.y + nj1.y
        };
        p.cp2 = {
          x: p.ob.cp2.x - j2.x + nj2.x,
          y: p.ob.cp2.y - j2.y + nj2.y
        };
      }

      let newP;
      if (p.shape === Path.Shape.STRAIGHT) {
        newP = makePathStraight(nj1, nj2);
      } else if (p.shape === Path.Shape.BEZIER) {
        newP = makePathBezier(nj1, nj2, p.cp1, p.cp2);
      }
      if (!newP) {
        return false;
      }

      p.cp1 = newP.cp1;
      p.cp2 = newP.cp2;
      p.distance = newP.distance;
      p.j1 = idxj1;
      p.j2 = idxj2;

      cln.p.push([nj1, nj2, p]);
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

  function makeLandmark(pt, ob) {
    /* create a new landmark without overlapping existing ones */
    /* the object being updated can be supplied as ob, for example when dragging */
    var lm = new Landmark({
      x: pt[0],
      y: pt[1]
    });
    lm.ob = ob;

    var hit = viz.models.hitTestLandmark(lm);
    if (!hit) {
      return lm;
    } else {
      return null;
    }
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
    var hit = viz.models.selectObject(viz.zoom.pixelToCoord(pt));
    return hit ? hit : null;
  }

  function snapToJunction(pt) {
    /* snap to an existing junction. */
    var j = viz.models.selectJunction(viz.zoom.pixelToCoord(pt));
    return j instanceof Junction ? j : null;
  }

  function snapToBezierCP(pt, path) {
    /* snap to the control points of a given bezier path. */
    var sd = viz.zoom.getSnapDistance();
    var hit;

    function selectBezierCP(ptTest, tolerance) {
      let ds1 = sqDist(ptTest, [path.cp1.x, path.cp1.y]);
      let ds2 = sqDist(ptTest, [path.cp2.x, path.cp2.y]);
      let radiusSum = tolerance + BEZIER_CP_RADIUS;
      let sqRadius = radiusSum * radiusSum;
      if (ds1 < sqRadius || ds2 < sqRadius) {
        return ds1 < ds2 ? 1 : 2;
      }
    }

    pt = viz.zoom.pixelToCoord(pt);
    if (!(path && path instanceof Path && path.shape === Path.Shape.BEZIER)) {
      return null;
    }
    hit = selectBezierCP(pt, sd);
    return hit ? hit : null;
  }

  function snapToLandmark(pt) {
    /* snap to an existing landmark. */
    var sd = viz.zoom.getSnapDistance();
    var lsd = viz.zoom.getLowerSnapDistance();
    var hit;

    if (lsd) {
      let _pt = snap(pt, lsd);
      hit = viz.models.hitTestLandmark(
        new Landmark({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    if (!hit) {
      let _pt = snap(pt, sd);
      hit = viz.models.hitTestLandmark(
        new Landmark({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    return hit ? hit : null;
  }

  function snapToLivaAgv(pt) {
    if (!viz.liveApp || !viz.liveApp.isPoseValid()) {
      return;
    }
    var lsd = viz.zoom.getLowerSnapDistance();
    var robotPose = viz.liveApp.rawRobotPose();
    var agvPt = [robotPose.x, robotPose.y];

    pt = snap(pt, lsd);

    var ds = sqDist(pt, agvPt);

    if (ds <= AGV_SNAP_DISTANCE * AGV_SNAP_DISTANCE) {
      return agvPt;
    }
  }

  function withinCorners(pt) {
    var corners = viz.zoom.getCorners();
    return (
      pt[0] >= corners.bottom &&
      pt[0] <= corners.top &&
      pt[1] >= corners.right &&
      pt[1] <= corners.left
    );
  }

  function thetaTwoPoints(pt1, pt2) {
    var dy = pt2[1] - pt1[1];
    var dx = pt2[0] - pt1[0];
    var theta = Math.atan2(dy, dx);
    return theta;
  }

  function sqDist(a, b) {
    var d0 = a[0] - b[0];
    var d1 = a[1] - b[1];
    return d0 * d0 + d1 * d1;
  }

  return {
    makeJunction: makeJunction,
    makePath: makePath,
    makeStation: makeStation,
    makeDraggables: makeDraggables,
    saveDraggables: saveDraggables,
    makeClone: makeClone,
    saveClone: saveClone,
    makeLandmark: makeLandmark,
    makeArea: makeArea,
    objectsInArea: objectsInArea,
    snap: snap,
    snapToObject: snapToObject,
    snapToJunction: snapToJunction,
    snapToBezierCP: snapToBezierCP,
    snapToLandmark: snapToLandmark,
    snapToLivaAgv: snapToLivaAgv,
    withinCorners: withinCorners,
    checkBezierLimits: checkBezierLimits,
    computeBezierPathLength: computeBezierPathLength,
    generateConnectorPath: generateConnectorPath,
    generateRulerPath: generateRulerPath,
    thetaTwoPoints: thetaTwoPoints
  };
}

/* Helper functions */
function checkBezierLimits(j1, j2, cp1, cp2, exclude) {
  // Check that the AGV is able to navigate the path.
  // Method: make sure when we project the points onto the line j1-j2,
  // the projected points are in the order [j1, cp1, cp2, j2].

  var v = [j2.x - j1.x, j2.y - j1.y];
  var v1 = [cp1.x - j1.x, cp1.y - j1.y];
  var v2 = [cp2.x - j1.x, cp2.y - j1.y];

  var dot = v[0] * v[0] + v[1] * v[1];
  var dot1 = v[0] * v1[0] + v[1] * v1[1];
  var dot2 = v[0] * v2[0] + v[1] * v2[1];

  if (exclude === 1) {
    return dot1 <= dot2 && dot2 <= dot;
  } else if (exclude === 2) {
    return 0 <= dot1 && dot1 <= dot2;
  }
  return 0 <= dot1 && dot1 <= dot2 && dot2 <= dot;
}

function computeBezierPathLength(path, j1, j2) {
  if (path.shape !== Path.Shape.BEZIER) {
    return 0;
  }

  // derivative formula: see http://stackoverflow.com/a/31317254
  var c1 = [
    j2.x - 3 * path.cp2.x + 3 * path.cp1.x - j1.x,
    j2.y - 3 * path.cp2.y + 3 * path.cp1.y - j1.y
  ];
  var c2 = [
    3 * path.cp2.x - 6 * path.cp1.x + 3 * j1.x,
    3 * path.cp2.y - 6 * path.cp1.y + 3 * j1.y
  ];
  var c3 = [3 * path.cp1.x - 3 * j1.x, 3 * path.cp1.y - 3 * j1.y];

  function derivative(t) {
    var t2 = t * t;
    return [3 * c1[0] * t2 + 2 * c2[0] * t + c3[0], 3 * c1[1] * t2 + 2 * c2[1] * t + c3[1]];
  }

  // approximation method
  var steps = 100;
  var dt = 1 / steps;
  var dx, dy;
  var sum = 0;
  for (var i = 0; i < steps; i++) {
    let t = (i + 0.5) * dt;
    [dx, dy] = derivative(t);
    sum += Math.sqrt(dx * dx + dy * dy);
  }
  sum *= dt;
  return sum;
}

/* SVG helper function */
function generateConnectorPath(path, j1, j2, trunkOnly) {
  var [flow, facing] = trunkOnly ? [0, 0] : [path.flow, path.facing];
  if (flow && facing >= Path.Facing.FORWARD_BI) {
    facing -= 2;
  }

  var extra = '';

  if (path.shape === Path.Shape.STRAIGHT) {
    if (flow || facing) {
      let headingRad = Math.atan2(j2.y - j1.y, j2.x - j1.x);
      extra = generateArrow([j1.x, j1.y], [j2.x, j2.y], headingRad, flow, facing);
    }
    return `M ${j1.x},${j1.y} L ${j2.x},${j2.y}${extra}`;
  } else if (path.shape === Path.Shape.BEZIER) {
    if (flow || facing) {
      let headingRad = Math.atan2(
        j2.y + path.cp2.y - path.cp1.y - j1.y,
        j2.x + path.cp2.x - path.cp1.x - j1.x
      );
      extra = generateArrow(
        [(j1.x + 3 * path.cp1.x) / 4, (j1.y + 3 * path.cp1.y) / 4],
        [(j2.x + 3 * path.cp2.x) / 4, (j2.y + 3 * path.cp2.y) / 4],
        headingRad,
        flow,
        facing
      );
    }
    return `M ${j1.x},${j1.y} C ${path.cp1.x},${path.cp1.y},${path.cp2.x},${path.cp2.y},${j2.x},${j2.y}${extra}`;
  }
}

function generateArrow(pt1, pt2, headingRad, flow, facing) {
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
  var cosH = Math.cos(headingRad);
  var sinH = Math.sin(headingRad);

  var extra = '';
  var pts;
  if (flow) {
    pts = [
      [-h, w],
      [h, 0],
      [-h, -w]
    ];
    pts = pts.map(rp);
    pts.push(pts[1]);
    pts.push(pts[0]); // repeat to fill dashed (dynamic) path
    extra += ' M' + pts.join('L');
  }

  pts = null;
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
    pts = pts.concat(pts, pts); // repeat to fill dashed (dynamic) path
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
    rot = (Math.atan2(v[0], v[1]) * 180) / Math.PI;
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
  var isSameJunction = idx1 === idx2;
  var stationsHeading = {};
  for (let station of stations) {
    // check if station overlap between j1 and j2
    if (indexes.includes(station.j)) {
      if (isSameJunction) {
        return false;
      }
      if (stationsHeading[station.heading]) {
        return false;
      }
      stationsHeading[station.heading] = true;
    }
  }
  return true;
}
