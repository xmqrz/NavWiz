/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Junction from './junction';
import Path from './path';
import Station from './station';

const JUNCTION_RADIUS = 0.001;
const STATION_RADIUS = 0.8 / 2 + 0.001; // 0.401
const STATION_HEADLESS_HIT = 0.201;
const PATH_WIDTH = 0.03;

function hitTestJunction(junctions, paths, testJ, tolerance, visibleLayers, excludeJ = []) {
  if (!Array.isArray(excludeJ)) {
    excludeJ = [excludeJ];
  }
  for (let junction of junctions) {
    if (junction === testJ || excludeJ.indexOf(junction) >= 0) {
      continue;
    }
    if (visibleLayers && !visibleLayers[junction.layer]) {
      continue;
    }
    if (hitTestJunctionAndJunction(junction, testJ)) {
      return junction;
    }
  }
  excludeJ = excludeJ.map((j) => junctions.indexOf(j));
  for (let path of paths) {
    if (excludeJ.indexOf(path.j1) >= 0 || excludeJ.indexOf(path.j2) >= 0) {
      continue;
    }
    if (visibleLayers && !visibleLayers[path.layer]) {
      continue;
    }
    if (
      path.j1 < 0 ||
      path.j1 >= junctions.length ||
      path.j2 < 0 ||
      path.j2 >= junctions.length
    ) {
      console.log("Path's junction index out of bounds.");
      continue;
    }
    let pathJ1 = junctions[path.j1];
    let pathJ2 = junctions[path.j2];
    if (hitTestJunctionAndPath(testJ, path, pathJ1, pathJ2, tolerance)) {
      return path;
    }
  }
  return false;
}

function hitTestPath(junctions, paths, testJ1, testJ2, testP, tolerance, excludeJ = []) {
  if (!Array.isArray(excludeJ)) {
    excludeJ = [excludeJ];
  }
  for (let junction of junctions) {
    if (excludeJ.indexOf(junction) >= 0) {
      continue;
    }
    if (hitTestJunctionAndPath(junction, testP, testJ1, testJ2, tolerance)) {
      return junction;
    }
  }
  excludeJ = excludeJ.map((j) => junctions.indexOf(j));
  for (let path of paths) {
    if (path === testP || excludeJ.indexOf(path.j1) >= 0 || excludeJ.indexOf(path.j2) >= 0) {
      continue;
    }
    if (
      path.j1 < 0 ||
      path.j1 >= junctions.length ||
      path.j2 < 0 ||
      path.j2 >= junctions.length
    ) {
      console.log("Path's junction index out of bounds.");
      continue;
    }
    let pathJ1 = junctions[path.j1];
    let pathJ2 = junctions[path.j2];
    if (
      (hitTestJunctionAndJunction(pathJ1, testJ1) &&
        hitTestJunctionAndJunction(pathJ2, testJ2)) ||
      (hitTestJunctionAndJunction(pathJ1, testJ2) &&
        hitTestJunctionAndJunction(pathJ2, testJ1))
    ) {
      return path;
    }
  }
  return false;
}

function selectObject(junctions, paths, stations, testPt, tolerance, visibleLayers) {
  // Select nearest element within tolerance

  // select junction
  var selectJ = selectJunction(junctions, testPt, tolerance, visibleLayers);
  if (selectJ) {
    return selectJ;
  }

  var hit = [];
  var testJ = new Junction({
    x: testPt[0],
    y: testPt[1]
  });

  // select path
  for (let path of paths) {
    if (visibleLayers && !visibleLayers[path.layer]) {
      continue;
    }
    if (
      path.j1 < 0 ||
      path.j1 >= junctions.length ||
      path.j2 < 0 ||
      path.j2 >= junctions.length
    ) {
      console.log("Path's junction index out of bounds.");
      continue;
    }
    let pathJ1 = junctions[path.j1];
    let pathJ2 = junctions[path.j2];
    let distanceSquared = sqDistCircleAndPath(testJ, tolerance, path, pathJ1, pathJ2);
    if (distanceSquared !== false) {
      if (!hit.length || distanceSquared < hit[0]) {
        hit[0] = distanceSquared;
        hit[1] = path;
      }
    }
  }

  if (hit.length) {
    return hit[1];
  }

  // select station
  var radiusSum = tolerance + STATION_RADIUS;
  var rSq = radiusSum * radiusSum;
  // Find nearest junction first
  for (let station of stations) {
    if (station.j < 0 || station.j >= junctions.length) {
      console.log("Station's junction index out of bounds.");
      continue;
    }
    if (visibleLayers && !visibleLayers[station.layer]) {
      continue;
    }
    let stationJ = junctions[station.j];
    if (stationJ === hit[1]) {
      continue;
    }
    let distanceSquared = sqDist(testPt, [stationJ.x, stationJ.y]);
    if (distanceSquared < rSq) {
      if (!hit.length || distanceSquared < hit[0]) {
        hit[0] = distanceSquared;
        hit[1] = stationJ;
      }
    }
  }
  if (hit.length) {
    // pick station which heading is more aligned
    let hitJ = hit[1];
    hit = [];

    let diffX = testPt[0] - hitJ.x;
    let diffY = testPt[1] - hitJ.y;
    let heading = (Math.atan2(diffY, diffX) * 180) / Math.PI;
    if (heading < 0) {
      heading += 360;
    }

    for (let station of stations) {
      if (station.j < 0 || station.j >= junctions.length) {
        console.log("Station's junction index out of bounds.");
        continue;
      }
      if (visibleLayers && !visibleLayers[station.layer]) {
        continue;
      }
      let stationJ = junctions[station.j];
      if (stationJ !== hitJ) {
        continue;
      }

      if (Math.abs(diffX) < STATION_HEADLESS_HIT && Math.abs(diffY) < STATION_HEADLESS_HIT) {
        if (hit.length && hit[1].heading === Station.Heading.NA) {
          continue;
        }
        if (station.heading === Station.Heading.NA) {
          hit[0] = 0;
          hit[1] = station;
          continue;
        }
      }

      let deltaHeading = Math.abs(station.heading - heading);
      if (deltaHeading > 180) {
        deltaHeading = 360 - deltaHeading;
      }
      if (!hit.length || deltaHeading < hit[0]) {
        hit[0] = deltaHeading;
        hit[1] = station;
      }
    }
  }

  if (hit.length) {
    return hit[1];
  }

  return false;
}

function selectJunction(junctions, testPt, tolerance, visibleLayers) {
  // Select nearest junction within tolerance
  var hit = [];
  var radiusSum = tolerance + JUNCTION_RADIUS;
  var rSq = radiusSum * radiusSum;
  for (let j of junctions) {
    if (visibleLayers && !visibleLayers[j.layer]) {
      continue;
    }
    let distanceSquared = sqDist(testPt, [j.x, j.y]);
    if (distanceSquared < rSq) {
      if (!hit.length || distanceSquared < hit[0]) {
        hit[0] = distanceSquared;
        hit[1] = j;
      }
    }
  }

  if (hit.length) {
    return hit[1];
  }
  return false;
}

function hitTestRect(junctions, paths, stations, testPt1, testPt2, tolerance, visibleLayers) {
  /* Return list of objects inside rect */
  var objects = [];

  var maxX = Math.max(testPt1[0], testPt2[0]);
  var maxY = Math.max(testPt1[1], testPt2[1]);
  var minX = Math.min(testPt1[0], testPt2[0]);
  var minY = Math.min(testPt1[1], testPt2[1]);

  // Junction
  objects = junctions.filter(
    (j) =>
      (!visibleLayers || visibleLayers[j.layer]) &&
      j.x >= minX - tolerance &&
      j.x <= maxX + tolerance &&
      j.y >= minY - tolerance &&
      j.y <= maxY + tolerance
  );

  // Path
  objects = objects.concat(
    paths.filter(function (p) {
      if (visibleLayers && !visibleLayers[p.layer]) {
        return false;
      }

      if (p.j1 < 0 || p.j2 < 0 || p.j1 >= junctions.length || p.j2 >= junctions.length) {
        console.log("Path's junction index out of bounds.");
        return false;
      }

      let j1 = junctions[p.j1];
      let j2 = junctions[p.j2];

      let junctionInArea =
        (j1.x >= minX && j1.x <= maxX && j1.y >= minY && j1.y <= maxY) ||
        (j2.x >= minX && j2.x <= maxX && j2.y >= minY && j2.y <= maxY);
      if (junctionInArea) {
        return true;
      }

      let rectJunctions = [
        new Junction({
          x: minX,
          y: minY
        }),
        new Junction({
          x: maxX,
          y: minY
        }),
        new Junction({
          x: maxX,
          y: maxY
        }),
        new Junction({
          x: minX,
          y: maxY
        })
      ];

      for (let i = 0; i < rectJunctions.length; i++) {
        if (hitTestJunctionAndPath(rectJunctions[i], p, j1, j2, tolerance)) {
          return true;
        }
        let testJ1 = rectJunctions[i];
        let testJ2 = rectJunctions[i + 1 >= rectJunctions.length ? 0 : i + 1];
        let testP = new Path({
          shape: Path.Shape.STRAIGHT
        });
        if (
          p.shape !== Path.Shape.BEZIER &&
          hitTestPathAndPath(testP, testJ1, testJ2, p, j1, j2)
        ) {
          return true;
        }
        if (
          p.shape === Path.Shape.BEZIER &&
          hitTestBezierAndLine(p, j1, j2, testP, testJ1, testJ2)
        ) {
          return true;
        }
      }
    })
  );

  // Station
  maxX = Math.max(testPt1[0] + STATION_RADIUS, testPt2[0] + STATION_RADIUS);
  maxY = Math.max(testPt1[1] + STATION_RADIUS, testPt2[1] + STATION_RADIUS);
  minX = Math.min(testPt1[0] - STATION_RADIUS, testPt2[0] - STATION_RADIUS);
  minY = Math.min(testPt1[1] - STATION_RADIUS, testPt2[1] - STATION_RADIUS);

  objects = objects.concat(
    stations.filter(function (s) {
      if (visibleLayers && !visibleLayers[s.layer]) {
        return false;
      }

      if (s.j < 0 || s.j >= junctions.length) {
        console.log("Station's junction index out of bounds.");
        return false;
      }
      var stationJ = junctions[s.j];
      return (
        stationJ.x >= minX && stationJ.x <= maxX && stationJ.y >= minY && stationJ.y <= maxY
      );
    })
  );

  return objects;
}

function hitTestLandmark(landmarks, testL) {
  for (let landmark of landmarks) {
    if (landmark === testL || landmark === testL.ob) {
      continue;
    }
    if (hitTestLandmarkAndLandmark(landmark, testL)) {
      return landmark;
    }
  }
  return false;
}

/* Helper functions */
function hitTestJunctionAndJunction(junction1, junction2) {
  /* Return true if the junctions overlap each other, false if otherwise. */
  if (junction1.x === junction2.x && junction1.y === junction2.y) {
    return true;
  }
  // Limit junction distance
  return hitTestCircleAndCircle(junction1, JUNCTION_RADIUS, junction2, JUNCTION_RADIUS);
}

function hitTestJunctionAndPath(junction, path, pathJ1, pathJ2, tolerance) {
  /**
   * Return true if junction lies on the path, false if otherwise.
   * On the edge case when the path only touches the junction, for eg: (-.),
   *   the result is false. Reason: This is legal - path connected to junction.
   */
  var minX, maxX, minY, maxY;
  [minX, maxX] = pathJ1.x < pathJ2.x ? [pathJ1.x, pathJ2.x] : [pathJ2.x, pathJ1.x];
  [minY, maxY] = pathJ1.y < pathJ2.y ? [pathJ1.y, pathJ2.y] : [pathJ2.y, pathJ1.y];

  // Make it easier for clicking if tolerance is very small.
  tolerance = Math.max(0.01, tolerance);

  // Filter the edge case when path only touches the junction.
  if (
    hitTestJunctionAndJunction(junction, pathJ1) ||
    hitTestJunctionAndJunction(junction, pathJ2)
  ) {
    return false;
  }

  if (path.shape === Path.Shape.STRAIGHT) {
    // Out-of-bounds
    if (junction.x < minX || junction.x > maxX || junction.y < minY || junction.y > maxY) {
      return false;
    }

    // Perpendicular distance to the straight line is within tolerance
    let v1 = [pathJ2.x - pathJ1.x, pathJ2.y - pathJ1.y];
    let v2 = [junction.x - pathJ1.x, junction.y - pathJ1.y];
    let v12 = crossProduct(v1, v2);
    if ((v12 * v12) / dotProduct(v1, v1) < tolerance * tolerance) {
      return true;
    }
  } else if (path.shape === Path.Shape.BEZIER) {
    // Out-of-bounds (Bezier)
    minX = Math.min(minX, path.cp1.x, path.cp2.x);
    maxX = Math.max(maxX, path.cp1.x, path.cp2.x);
    minY = Math.min(minY, path.cp1.y, path.cp2.y);
    maxY = Math.max(maxY, path.cp1.y, path.cp2.y);
    if (junction.x < minX || junction.x > maxX || junction.y < minY || junction.y > maxY) {
      return false;
    }

    // interpolation formula: see http://stackoverflow.com/a/31317254
    let c1 = [
      pathJ2.x - 3 * path.cp2.x + 3 * path.cp1.x - pathJ1.x,
      pathJ2.y - 3 * path.cp2.y + 3 * path.cp1.y - pathJ1.y
    ];
    let c2 = [
      3 * path.cp2.x - 6 * path.cp1.x + 3 * pathJ1.x,
      3 * path.cp2.y - 6 * path.cp1.y + 3 * pathJ1.y
    ];
    let c3 = [3 * path.cp1.x - 3 * pathJ1.x, 3 * path.cp1.y - 3 * pathJ1.y];
    let c4 = [pathJ1.x, pathJ1.y];

    let interpolate = function (t) {
      var t2 = t * t;
      var t3 = t2 * t;
      return [
        c1[0] * t3 + c2[0] * t2 + c3[0] * t + c4[0],
        c1[1] * t3 + c2[1] * t2 + c3[1] * t + c4[1]
      ];
    };

    // binary search for shortest distance
    // Todo: Fix the inaccuracies in the calculation.
    let pt = [junction.x, junction.y];
    let sqTolerance = tolerance * tolerance;

    let t0 = 0;
    let t1 = 1;
    let pt0 = interpolate(t0);
    let pt1 = interpolate(t1);
    let d0 = sqDist(pt0, pt);
    let d1 = sqDist(pt1, pt);

    while (sqDist(pt0, pt1) >= sqTolerance / 16) {
      if (d0 < d1) {
        if (d0 < sqTolerance) {
          return true;
        }
        t1 = (t0 + t1) / 2;
        pt1 = interpolate(t1);
        d1 = sqDist(pt1, pt);
      } else {
        if (d1 < sqTolerance) {
          return true;
        }
        t0 = (t0 + t1) / 2;
        pt0 = interpolate(t0);
        d0 = sqDist(pt0, pt);
      }
    }
  }

  return false;
}

function hitTestPathAndPath(path1, path1J1, path1J2, path2, path2J1, path2J2) {
  /**
   * For case when both paths are straight:
   * Return true if the paths cross or overlap each other, false if otherwise.
   * On the edge case when the paths only touch at the edge point, for eg: (_|, --),
   *   the result is false. Reason: This is legal - paths connected end-to-end.
   * On the edge case then the paths only touch at a single point, for eg: (-|, -(),
   *   the result is not consistent, i.e:
   *     - false for straight-to-straight,
   *   This is fine since hitTestJunctionAndPath() would have caught this.
   *
   * For case when one or both paths are bezier:
   * Cross and overlap does not matter.
   * Return true unless both endpoints of both paths overlap.
   */

  // Swap path so that we have fewer conditional logic branches.
  if (path1.shape < path2.shape) {
    [path1, path2] = [path2, path1];
    [path1J1, path2J1] = [path2J1, path1J1];
    [path1J2, path2J2] = [path2J2, path1J2];
  }

  var min1X, max1X, min1Y, max1Y;
  var min2X, max2X, min2Y, max2Y;
  [min1X, max1X] = path1J1.x < path1J2.x ? [path1J1.x, path1J2.x] : [path1J2.x, path1J1.x];
  [min1Y, max1Y] = path1J1.y < path1J2.y ? [path1J1.y, path1J2.y] : [path1J2.y, path1J1.y];
  [min2X, max2X] = path2J1.x < path2J2.x ? [path2J1.x, path2J2.x] : [path2J2.x, path2J1.x];
  [min2Y, max2Y] = path2J1.y < path2J2.y ? [path2J1.y, path2J2.y] : [path2J2.y, path2J1.y];

  if (path1.shape === Path.Shape.STRAIGHT) {
    if (path2.shape === Path.Shape.STRAIGHT) {
      // filter out non-overlapped parallel paths because the algorithm below couldn't.
      if ((min1X >= max2X || max1X <= min2X) && (min1Y >= max2Y || max1Y <= min2Y)) {
        return false;
      }

      // check intersection between two lines. See http://stackoverflow.com/a/30160064
      let v1 = [path1J2.x - path1J1.x, path1J2.y - path1J1.y];
      let v2 = [path2J2.x - path2J1.x, path2J2.y - path2J1.y];
      let v11 = [path2J1.x - path1J1.x, path2J1.y - path1J1.y];
      let v12 = [path2J2.x - path1J1.x, path2J2.y - path1J1.y];
      let v21 = [path1J1.x - path2J1.x, path1J1.y - path2J1.y];
      let v22 = [path1J2.x - path2J1.x, path1J2.y - path2J1.y];

      let c11 = crossProduct(v1, v11);
      let c12 = crossProduct(v1, v12);
      let c21 = crossProduct(v2, v21);
      let c22 = crossProduct(v2, v22);

      // intersects, touching doesn't count.
      if (c11 * c12 < 0 && c21 * c22 < 0) {
        return true;
      }
      // overlaps in parallel.
      if (c11 === 0 && c12 === 0 && c21 === 0 && c22 === 0) {
        return true;
      }
    } else {
      console.log('Path2 has invalid shape.');
    }
  } else if (path1.shape === Path.Shape.BEZIER) {
    if (path2.shape === Path.Shape.STRAIGHT || path2.shape === Path.Shape.BEZIER) {
      return (
        (hitTestJunctionAndJunction(path1J1, path2J1) &&
          hitTestJunctionAndJunction(path1J2, path2J2)) ||
        (hitTestJunctionAndJunction(path1J1, path2J2) &&
          hitTestJunctionAndJunction(path1J2, path2J1))
      );
    } else {
      console.log('Path2 has invalid shape.');
    }
  } else {
    console.log('Path1 has invalid shape.');
  }

  return false;
}

function hitTestBezierAndLine(bezier, bezierJ1, bezierJ2, line, lineJ1, lineJ2) {
  var minX, maxX, minY, maxY;
  [minX, maxX] = bezierJ1.x < bezierJ2.x ? [bezierJ1.x, bezierJ2.x] : [bezierJ2.x, bezierJ1.x];
  [minY, maxY] = bezierJ1.y < bezierJ2.y ? [bezierJ1.y, bezierJ2.y] : [bezierJ2.y, bezierJ1.y];
  minX = Math.min(minX, bezier.cp1.x, bezier.cp2.x);
  maxX = Math.max(maxX, bezier.cp1.x, bezier.cp2.x);
  minY = Math.min(minY, bezier.cp1.y, bezier.cp2.y);
  maxY = Math.max(maxY, bezier.cp1.y, bezier.cp2.y);

  var rectJunctions = [
    new Junction({
      x: minX,
      y: minY
    }),
    new Junction({
      x: maxX,
      y: minY
    }),
    new Junction({
      x: maxX,
      y: maxY
    }),
    new Junction({
      x: minX,
      y: maxY
    })
  ];

  var inBound =
    (lineJ1.x >= minX && lineJ1.x <= maxX && lineJ1.y >= minY && lineJ1.y <= maxY) ||
    (lineJ2.x >= minX && lineJ2.x <= maxX && lineJ2.y >= minY && lineJ2.y <= maxY);

  if (!inBound) {
    for (let i = 0; i < rectJunctions.length; i++) {
      let testJ1 = rectJunctions[i];
      let testJ2 = rectJunctions[i + 1 >= rectJunctions.length ? 0 : i + 1];
      let testP = new Path({
        shape: Path.Shape.STRAIGHT
      });
      if (hitTestPathAndPath(testP, testJ1, testJ2, line, lineJ1, lineJ2)) {
        inBound = true;
        break;
      }
    }
  }

  if (!inBound) {
    return false;
  }

  return (
    intersectionBezierAndLine(bezier, bezierJ1, bezierJ2, line, lineJ1, lineJ2).length > 0
  );
}

function hitTestLandmarkAndLandmark(landmark1, landmark2) {
  /* Return true if the landmarks are less then 0.2m apart, false if otherwise. */
  return sqDist([landmark1.x, landmark1.y], [landmark2.x, landmark2.y]) <= 0.2 * 0.2 - 0.001;
}

function hitTestCircleAndCircle(j1, r1, j2, r2) {
  let radiusSum = r1 + r2;
  return sqDist([j1.x, j1.y], [j2.x, j2.y]) < radiusSum * radiusSum;
}

function sqDistCircleAndPath(c, cr, path, pathJ1, pathJ2) {
  // Return sqDist for circle and path if overlap otherwise return false
  var minX, maxX, minY, maxY;
  [minX, maxX] = pathJ1.x < pathJ2.x ? [pathJ1.x, pathJ2.x] : [pathJ2.x, pathJ1.x];
  [minY, maxY] = pathJ1.y < pathJ2.y ? [pathJ1.y, pathJ2.y] : [pathJ2.y, pathJ1.y];

  // Filter the edge case when only path junction touch the circle.
  if (
    hitTestCircleAndCircle(c, cr, pathJ1, JUNCTION_RADIUS) ||
    hitTestCircleAndCircle(c, cr, pathJ2, JUNCTION_RADIUS)
  ) {
    return false;
  }

  if (path.shape === Path.Shape.STRAIGHT) {
    // Out-of-bounds
    if (c.x + cr < minX || c.x - cr > maxX || c.y + cr < minY || c.y - cr > maxY) {
      return false;
    }

    // Perpendicular distance to the straight line is within tolerance
    let v1 = [pathJ2.x - pathJ1.x, pathJ2.y - pathJ1.y];
    let v2 = [c.x - pathJ1.x, c.y - pathJ1.y];
    let v12 = crossProduct(v1, v2);
    let distanceSquared = (v12 * v12) / dotProduct(v1, v1);
    if (distanceSquared < cr * cr) {
      return distanceSquared;
    }
  } else if (path.shape === Path.Shape.BEZIER) {
    // Out-of-bounds (Bezier)
    minX = Math.min(minX, path.cp1.x, path.cp2.x);
    maxX = Math.max(maxX, path.cp1.x, path.cp2.x);
    minY = Math.min(minY, path.cp1.y, path.cp2.y);
    maxY = Math.max(maxY, path.cp1.y, path.cp2.y);
    if (c.x + cr < minX || c.x - cr > maxX || c.y + cr < minY || c.y - cr > maxY) {
      return false;
    }

    // interpolation formula: see http://stackoverflow.com/a/31317254
    let c1 = [
      pathJ2.x - 3 * path.cp2.x + 3 * path.cp1.x - pathJ1.x,
      pathJ2.y - 3 * path.cp2.y + 3 * path.cp1.y - pathJ1.y
    ];
    let c2 = [
      3 * path.cp2.x - 6 * path.cp1.x + 3 * pathJ1.x,
      3 * path.cp2.y - 6 * path.cp1.y + 3 * pathJ1.y
    ];
    let c3 = [3 * path.cp1.x - 3 * pathJ1.x, 3 * path.cp1.y - 3 * pathJ1.y];
    let c4 = [pathJ1.x, pathJ1.y];

    let interpolate = function (t) {
      var t2 = t * t;
      var t3 = t2 * t;
      return [
        c1[0] * t3 + c2[0] * t2 + c3[0] * t + c4[0],
        c1[1] * t3 + c2[1] * t2 + c3[1] * t + c4[1]
      ];
    };

    // sample search then binary search;
    let pt = [c.x, c.y];
    let radiusSum = cr + PATH_WIDTH;
    let sqRadius = radiusSum * radiusSum;

    // sample search
    let sample = 10;
    let idxmid = 0;
    let tmid = 0;
    let ptmid = interpolate(tmid);
    let dmid = sqDist(ptmid, pt);
    for (let index = 1; index <= sample; index++) {
      let tsample = index / sample;
      let ptsample = interpolate(tsample);
      let dsample = sqDist(ptsample, pt);
      if (dsample < dmid) {
        idxmid = index;
        tmid = tsample;
        ptmid = ptsample;
        dmid = dsample;
      }
    }
    // binary search for shortest distance
    // Todo: Fix the inaccuracies in the calculation.
    let t0 = Math.max((idxmid - 1) / sample, 0);
    let t1 = Math.min((idxmid + 1) / sample, 1);
    let pt0 = interpolate(t0);
    let pt1 = interpolate(t1);
    let d0 = sqDist(pt0, pt);
    let d1 = sqDist(pt1, pt);

    while (sqDist(pt0, pt1) >= sqRadius / 16) {
      if (d0 < d1) {
        if (d0 < sqRadius) {
          return d0;
        }
        t1 = (t0 + t1) / 2;
        pt1 = interpolate(t1);
        d1 = sqDist(pt1, pt);
      } else {
        if (d1 < sqRadius) {
          return d1;
        }
        t0 = (t0 + t1) / 2;
        pt0 = interpolate(t0);
        d0 = sqDist(pt0, pt);
      }
    }
  }

  return false;
}

// https://mathhelpforum.com/threads/solving-intersections-between-a-bezier-curve-and-straight-line.276396/
function intersectionBezierAndLine(bezierP, bezierJ1, bezierJ2, lineP, lineJ1, lineJ2) {
  /* Return list of intersections point */
  var A = lineJ2.y - lineJ1.y;
  var B = lineJ1.x - lineJ2.x;
  var C = lineJ1.x * (lineJ1.y - lineJ2.y) + lineJ1.y * (lineJ2.x - lineJ1.x);

  var bx = bezierCoeffs(bezierJ1.x, bezierP.cp1.x, bezierP.cp2.x, bezierJ2.x);
  var by = bezierCoeffs(bezierJ1.y, bezierP.cp1.y, bezierP.cp2.y, bezierJ2.y);

  var P = [];
  P[0] = A * bx[0] + B * by[0];
  P[1] = A * bx[1] + B * by[1];
  P[2] = A * bx[2] + B * by[2];
  P[3] = A * bx[3] + B * by[3] + C;

  var r = cubicRoots(P);

  var intersections = [];

  for (let i = 0; i < r.length; i++) {
    let t = r[i];
    let pt = [];

    pt = {
      x: bx[0] * t * t * t + bx[1] * t * t + bx[2] * t + bx[3],
      y: by[0] * t * t * t + by[1] * t * t + by[2] * t + by[3]
    };

    let s;
    if (lineJ2.x - lineJ1.x !== 0) {
      // if not vertical line
      s = (pt.x - lineJ1.x) / (lineJ2.x - lineJ1.x);
    } else {
      s = (pt.y - lineJ1.y) / (lineJ2.y - lineJ1.y);
    }

    // skip out bounds
    if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0) {
      continue;
    }
    intersections.push(pt);
  }
  return intersections;
}

function cubicRoots(P) {
  var a = P[0];
  var b = P[1];
  var c = P[2];
  var d = P[3];

  var A = b / a;
  var B = c / a;
  var C = d / a;

  var Im;

  var Q = (3.0 * B - Math.pow(A, 2.0)) / 9.0;
  var R = (9.0 * A * B - 27.0 * C - 2.0 * Math.pow(A, 3.0)) / 54.0;
  var D = Math.pow(Q, 3.0) + Math.pow(R, 2.0); // polynomial discriminant

  var t = [];

  if (D >= 0) {
    // complex or duplicate roots POI
    var S = Math.sign(R + Math.sqrt(D)) * Math.pow(Math.abs(R + Math.sqrt(D)), 1 / 3);
    var T = Math.sign(R - Math.sqrt(D)) * Math.pow(Math.abs(R - Math.sqrt(D)), 1 / 3);

    t[0] = -A / 3.0 + (S + T); // real root
    t[1] = -A / 3.0 - (S + T) / 2.0; // real part of complex root
    t[2] = -A / 3.0 - (S + T) / 2.0; // real part of complex root
    Im = Math.abs((Math.sqrt(3.0) * (S - T)) / 2.0); // complex part of root pair

    // discard complex roots
    if (Im !== 0) {
      t = [t[0]];
    }
  } else {
    // distinct real roots
    var th = Math.acos(R / Math.sqrt(-Math.pow(Q, 3)));

    t[0] = 2.0 * Math.sqrt(-Q) * Math.cos(th / 3) - A / 3;
    t[1] = 2.0 * Math.sqrt(-Q) * Math.cos((th + 2 * Math.PI) / 3) - A / 3;
    t[2] = 2.0 * Math.sqrt(-Q) * Math.cos((th + 4 * Math.PI) / 3) - A / 3;
    Im = 0.0;
  }

  // discard out of spec roots
  return t.filter((n) => n > 0.0 && n < 1.0);
}

function bezierCoeffs(p0, p1, p2, p3) {
  var z = [];
  z[0] = -p0 + 3 * p1 + -3 * p2 + p3;
  z[1] = 3 * p0 - 6 * p1 + 3 * p2;
  z[2] = -3 * p0 + 3 * p1;
  z[3] = p0;
  return z;
}

function sqDist(a, b) {
  var d0 = a[0] - b[0];
  var d1 = a[1] - b[1];
  return d0 * d0 + d1 * d1;
}

function dotProduct(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

function crossProduct(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

export default {
  /* Hit Test */
  junction: hitTestJunction,
  path: hitTestPath,
  rect: hitTestRect,
  landmark: hitTestLandmark,
  selectObject: selectObject,
  selectJunction: selectJunction
};
