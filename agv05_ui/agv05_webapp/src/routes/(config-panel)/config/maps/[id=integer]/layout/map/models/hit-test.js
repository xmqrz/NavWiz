/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Junction from './junction';
import Path from './path';
import Station from './station';

function hitTestJunction(junctions, paths, testJ, excludeJ = []) {
  if (!Array.isArray(excludeJ)) {
    excludeJ = [excludeJ];
  }
  for (let junction of junctions) {
    if (junction === testJ || excludeJ.indexOf(junction) >= 0) {
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
    if (hitTestJunctionAndPath(testJ, path, pathJ1, pathJ2)) {
      return path;
    }
  }
  return false;
}

function hitTestPath(junctions, paths, testJ1, testJ2, testP, excludeJ = []) {
  if (!Array.isArray(excludeJ)) {
    excludeJ = [excludeJ];
  }
  for (let junction of junctions) {
    if (excludeJ.indexOf(junction) >= 0) {
      continue;
    }
    if (hitTestJunctionAndPath(junction, testP, testJ1, testJ2)) {
      return junction;
    }
  }
  let branched = {
    bj1: false,
    bj2: false
  };
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
    if (hitTestPathAndPath(path, pathJ1, pathJ2, testP, testJ1, testJ2)) {
      let branch = branchTest(paths, path, pathJ1, pathJ2, testP, testJ1, testJ2);
      if (!branch || branched[branch]) {
        return path;
      } else {
        branched[branch] = true;
      }
    }
  }

  return false;
}

//return 'bj1' or 'bj2' if can branch
function branchTest(paths, path, pathJ1, pathJ2, testP, testPJ1, testPJ2) {
  let isCommonPathJ1 = [testPJ1, testPJ2].indexOf(pathJ1) >= 0;
  let isCommonPathJ2 = [testPJ1, testPJ2].indexOf(pathJ2) >= 0;

  if (!isCommonPathJ1 && !isCommonPathJ2) {
    return false;
  } else if (isCommonPathJ1 && isCommonPathJ2) {
    return false;
  }
  let commonJ = isCommonPathJ1 ? pathJ1 : pathJ2;
  var pathD = isCommonPathJ1
    ? ['bj1', path.direction, pathJ2]
    : ['bj2', Path.getReverseDirection(path), pathJ1];
  var testD =
    testPJ1 === commonJ
      ? ['bj1', testP.direction, testPJ2]
      : ['bj2', Path.getReverseDirection(testP), testPJ1];

  // check is same direction
  if (pathD[1] !== testD[1]) {
    return false;
  }

  // check s_curve and bend path case
  if (
    [path.shape, testP.shape].indexOf(Path.Shape.S_CURVE) >= 0 &&
    (path.shape > Path.Shape.S_CURVE || testP.shape > Path.Shape.S_CURVE)
  ) {
    let sCurveP = path.shape === Path.Shape.S_CURVE ? path : testP;
    let sCurveJ = sCurveP === path ? pathD[2] : testD[2];
    let bendP = path.shape > Path.Shape.S_CURVE ? path : testP;
    let bendJ = bendP === path ? pathD[2] : testD[2];

    // normalize coordinate bend in x direction(East or West)
    let x = Path.Direction.isEW(pathD[1]) ? 'x' : 'y';
    let y = x === 'y' ? 'x' : 'y';

    // if same side
    if (sCurveJ[y] > commonJ[y] === bendJ[y] > commonJ[y]) {
      // if scurve higher than bend
      if (Math.abs(sCurveJ[x] - commonJ[x]) > Math.abs(bendJ[x] - commonJ[x])) {
        return false;
      }
    }
  }

  return testD[0];
}

function hitTestPoint(junctions, paths, stations, testPt) {
  var hit = hitTestJunction(
    junctions,
    paths,
    new Junction({
      x: testPt[0],
      y: testPt[1]
    })
  );
  if (hit) {
    return hit;
  }

  var hitJ = null;
  var halfSize = 1.2 / 2 + 0.001;
  for (let station of stations) {
    if (station.j < 0 || station.j >= junctions.length) {
      console.log("Station's junction index out of bounds.");
      continue;
    }
    let stationJ = junctions[station.j];
    if (
      testPt[0] > stationJ.x - halfSize &&
      testPt[0] < stationJ.x + halfSize &&
      testPt[1] > stationJ.y - halfSize &&
      testPt[1] < stationJ.y + halfSize
    ) {
      let resetJ = false;
      if (!hitJ) {
        resetJ = true;
      } else if (hitJ.id !== station.j) {
        // pick station which junction is nearest
        let hitDist = sqDist([hitJ.x, hitJ.y], testPt);
        let newDist = sqDist([stationJ.x, stationJ.y], testPt);
        if (hitDist > newDist || (hitDist === newDist && hitJ.id > station.j)) {
          resetJ = true;
        }
      } else if (hit.direction !== station.direction) {
        // pick station which direction is more aligned
        let diffX = testPt[0] - hitJ.x;
        let diffY = testPt[1] - hitJ.y;
        let absDiffX = Math.abs(diffX);
        let absDiffY = Math.abs(diffY);
        let preference;

        if (absDiffX < 0.201 && absDiffY < 0.201) {
          preference = [1, 1, 1, 1, 0];
        } else {
          preference = [0, 0, 0, 0, 4];
        }

        if (absDiffX < absDiffY) {
          if (diffY <= 0) {
            preference[Station.Direction.SOUTH] += 3;
          } else {
            preference[Station.Direction.NORTH] += 3;
          }
          if (diffX >= 0) {
            preference[Station.Direction.EAST] += 1;
            preference[Station.Direction.WEST] += 2;
          } else {
            preference[Station.Direction.WEST] += 1;
            preference[Station.Direction.EAST] += 2;
          }
        } else {
          if (diffX >= 0) {
            preference[Station.Direction.WEST] += 3;
          } else {
            preference[Station.Direction.EAST] += 3;
          }
          if (diffY <= 0) {
            preference[Station.Direction.NORTH] += 1;
            preference[Station.Direction.SOUTH] += 2;
          } else {
            preference[Station.Direction.SOUTH] += 1;
            preference[Station.Direction.NORTH] += 2;
          }
        }

        if (preference[station.direction] < preference[hit.direction]) {
          hit = station;
        }
      }

      if (resetJ) {
        hit = station;
        hitJ = {
          id: station.j,
          x: stationJ.x,
          y: stationJ.y
        };
      }
    }
  }

  return hit;
}

function hitTestRect(junctions, paths, stations, testPt1, testPt2, tolerance) {
  /* Return list of objects inside rect */
  var objects = [];

  var maxX = Math.max(testPt1[0], testPt2[0]);
  var maxY = Math.max(testPt1[1], testPt2[1]);
  var minX = Math.min(testPt1[0], testPt2[0]);
  var minY = Math.min(testPt1[1], testPt2[1]);

  // Junction
  objects = junctions.filter(
    (j) =>
      j.x >= minX - tolerance &&
      j.x <= maxX + tolerance &&
      j.y >= minY - tolerance &&
      j.y <= maxY + tolerance
  );

  // Path
  objects = objects.concat(
    paths.filter(function (p) {
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
        if (hitTestPathAndPath(testP, testJ1, testJ2, p, j1, j2)) {
          return true;
        }
      }
    })
  );

  // Station
  var halfSize = 1.2 / 2 + 0.001;
  maxX = Math.max(testPt1[0] + halfSize, testPt2[0] + halfSize);
  maxY = Math.max(testPt1[1] + halfSize, testPt2[1] + halfSize);
  minX = Math.min(testPt1[0] - halfSize, testPt2[0] - halfSize);
  minY = Math.min(testPt1[1] - halfSize, testPt2[1] - halfSize);

  objects = objects.concat(
    stations.filter(function (s) {
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

/* Helper functions */
function hitTestJunctionAndJunction(junction1, junction2) {
  /* Return true if the junctions overlap each other, false if otherwise. */
  return junction1.x === junction2.x && junction1.y === junction2.y;
}

function hitTestJunctionAndPath(junction, path, pathJ1, pathJ2) {
  /**
   * Return true if junction lies on the path, false if otherwise.
   * On the edge case when the path only touches the junction, for eg: (-.),
   *   the result is false. Reason: This is legal - path connected to junction.
   */
  var minX, maxX, minY, maxY;
  [minX, maxX] = pathJ1.x < pathJ2.x ? [pathJ1.x, pathJ2.x] : [pathJ2.x, pathJ1.x];
  [minY, maxY] = pathJ1.y < pathJ2.y ? [pathJ1.y, pathJ2.y] : [pathJ2.y, pathJ1.y];

  if (path.shape === Path.Shape.STRAIGHT) {
    // Hit the straight line
    if (
      (junction.x === minX && junction.y > minY && junction.y < maxY) ||
      (junction.x > minX && junction.x < maxX && junction.y === minY)
    ) {
      return true;
    }
  } else if (path.shape === Path.Shape.S_CURVE) {
    // Hit within the bounding box of the S curve.
    if (Path.isNS(path)) {
      if (junction.x >= minX && junction.x <= maxX && junction.y > minY && junction.y < maxY) {
        return true;
      }
    } else if (Path.isEW(path)) {
      if (junction.x > minX && junction.x < maxX && junction.y >= minY && junction.y <= maxY) {
        return true;
      }
    } else {
      console.log('Path has invalid direction.');
      return false;
    }
  } else if (path.shape === Path.Shape.BEND_LEFT || path.shape === Path.Shape.BEND_RIGHT) {
    // Filter the edge case when path only touches the junction.
    if (
      hitTestJunctionAndJunction(junction, pathJ1) ||
      hitTestJunctionAndJunction(junction, pathJ2)
    ) {
      return false;
    }

    // Hit either strip of the right-angled path.
    let stripX, stripY;
    if (Path.isNS(path)) {
      stripX = pathJ1.x;
      stripY = pathJ2.y;
    } else if (Path.isEW(path)) {
      stripX = pathJ2.x;
      stripY = pathJ1.y;
    } else {
      console.log('Path has invalid direction.');
      return false;
    }
    if (
      (junction.x === stripX && junction.y >= minY && junction.y <= maxY) ||
      (junction.x >= minX && junction.x <= maxX && junction.y === stripY)
    ) {
      return true;
    }

    // Hit the corner radius.
    let radius = Math.min(maxX - minX, maxY - minY) / 2.0;
    radius = Math.min(radius, 1.0);
    let corners = {};
    switch (path.direction) {
      case Path.Direction.NORTH:
        corners.top = pathJ2.y;
        corners.left = pathJ1.x - (path.shape === Path.Shape.BEND_LEFT ? radius : 0);
        break;
      case Path.Direction.SOUTH:
        corners.top = pathJ2.y - radius;
        corners.left = pathJ1.x - (path.shape === Path.Shape.BEND_RIGHT ? radius : 0);
        break;
      case Path.Direction.EAST:
        corners.top = pathJ1.y - (path.shape === Path.Shape.BEND_LEFT ? radius : 0);
        corners.left = pathJ2.x - radius;
        break;
      case Path.Direction.WEST:
        corners.top = pathJ1.y - (path.shape === Path.Shape.BEND_RIGHT ? radius : 0);
        corners.left = pathJ2.x;
        break;
    }
    if (
      junction.x >= corners.left &&
      junction.x <= corners.left + radius &&
      junction.y >= corners.top &&
      junction.y <= corners.top + radius
    ) {
      return true;
    }
  } else {
    console.log('Path has invalid shape.');
  }

  return false;
}

function hitTestPathAndPath(path1, path1J1, path1J2, path2, path2J1, path2J2) {
  /**
   * Return true if the paths cross or overlap each other, false if otherwise.
   * On the edge case when the paths only touch at the edge point, for eg: (_|, --),
   *   the result is false. Reason: This is legal - paths connected end-to-end.
   * On the edge case then the paths only touch at a single point, for eg: (-|, -(),
   *   the result is not consistent, i.e:
   *     - false for straight-to-straight,
   *     - false for s_curve-to-straight,
   *     - false for s_curve-to-s_curve,
   *     - false for right_angled-to-straight (including touching the corner of right_angled),
   *     - false for right_angled-to-s_curve (including touching the corner of right_angled),
   *     - false for right_angled-to-right_angled (including touching the corner of right_angled),
   *   This is fine since hitTestJunctionAndPath() would have caught this.
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

  // Hack: This is heavily optimized.
  if (path1.shape === Path.Shape.STRAIGHT || path1.shape === Path.Shape.S_CURVE) {
    if (path2.shape === Path.Shape.STRAIGHT || path2.shape === Path.Shape.S_CURVE) {
      // Cross over perpendicularly between both straight lines, or
      // Cross over from straight line into S curve's region, or
      // Cross over between both S curves' regions.
      if (min1X < max2X && max1X > min2X && min1Y < max2Y && max1Y > min2Y) {
        return true;
      }
      // Overlap some sections parallelly.
      if (
        (Path.isNS(path1) &&
          Path.isNS(path2) &&
          (min1X === max2X || max1X === min2X) &&
          min1Y < max2Y &&
          max1Y > min2Y) ||
        (Path.isEW(path1) &&
          Path.isEW(path2) &&
          min1X < max2X &&
          max1X > min2X &&
          (min1Y === max2Y || max1Y === min2Y))
      ) {
        return true;
      }
    } else {
      console.log('Path2 has invalid shape.');
    }
  } else if (path1.shape === Path.Shape.BEND_LEFT || path1.shape === Path.Shape.BEND_RIGHT) {
    // Pre-compute each strip for path1.
    let strip1X, strip1Y;
    if (Path.isNS(path1)) {
      strip1X = path1J1.x;
      strip1Y = path1J2.y;
    } else if (Path.isEW(path1)) {
      strip1X = path1J2.x;
      strip1Y = path1J1.y;
    } else {
      console.log('Path1 has invalid direction.');
      return false;
    }

    if (path2.shape === Path.Shape.STRAIGHT || path2.shape === Path.Shape.S_CURVE) {
      // Cross over either strip of the right-angled path perpendicularly.
      if (
        (min1X < max2X && max1X > min2X && strip1Y < max2Y && strip1Y > min2Y) ||
        (strip1X < max2X && strip1X > min2X && min1Y < max2Y && max1Y > min2Y)
      ) {
        return true;
      }

      // Overlaps some section of the strip parallelly.
      if (Path.isNS(path2)) {
        if ((strip1X === min2X || strip1X === max2X) && min1Y < max2Y && max1Y > min2Y) {
          return true;
        }
      } else if (Path.isEW(path2)) {
        if (min1X < max2X && max1X > min2X && (strip1Y === min2Y || strip1Y === max2Y)) {
          return true;
        }
      } else {
        console.log('Path2 has invalid direction.');
        return false;
      }
    } else if (path2.shape === Path.Shape.BEND_LEFT || path2.shape === Path.Shape.BEND_RIGHT) {
      // Pre-compute each strip for path2.
      let strip2X, strip2Y;
      if (Path.isNS(path2)) {
        strip2X = path2J1.x;
        strip2Y = path2J2.y;
      } else if (Path.isEW(path2)) {
        strip2X = path2J2.x;
        strip2Y = path2J1.y;
      } else {
        console.log('Path2 has invalid direction.');
        return false;
      }

      // Cross over between either strip of the two right-angled paths.
      if (
        (min1X < strip2X && max1X > strip2X && strip1Y < max2Y && strip1Y > min2Y) ||
        (strip1X < max2X && strip1X > min2X && min1Y < strip2Y && max1Y > strip2Y)
      ) {
        return true;
      }

      // Overlaps some section of the strip parallelly.
      if (
        (strip1X === strip2X && min1Y < max2Y && max1Y > min2Y) ||
        (min1X < max2X && max1X > min2X && strip1Y === strip2Y)
      ) {
        return true;
      }
    } else {
      console.log('Path2 has invalid shape.');
    }
  } else {
    console.log('Path1 has invalid shape.');
  }

  return false;
}

function hitTestRegionPerimeter(region, pt, tolerance, limiter) {
  var tolSq = tolerance * tolerance;
  var a0 = region[region.length - 1];
  for (let i = 0; i < region.length; a0 = region[i++]) {
    let v1 = subtractV(region[i], a0);
    let v2 = subtractV(pt, a0);
    let c12 = crossProduct(v1, v2);
    let d11 = dotProduct(v1, v1);
    if ((c12 * c12) / d11 < tolSq) {
      let d12 = dotProduct(v1, v2);
      let proj21 = d12 / d11;
      if (proj21 <= 0 || proj21 >= 1) {
        continue;
      }

      let newPt = addV(a0, multV(v1, proj21));
      // discard if overlap with the corners.
      if (sqDist(newPt, a0) < tolSq * 4 || sqDist(newPt, region[i]) < tolSq * 4) {
        continue;
      }
      if (limiter && !limiter(newPt)) {
        continue;
      }
      return [newPt, i === 0 ? region.length : i];
    }
  }
  return [false, false];
}

function hitTestLineSegmentsIntersect(p1, p2, q1, q2) {
  var r = subtractV(p2, p1);
  var s = subtractV(q2, q1);

  var numerator = crossProduct(subtractV(q1, p1), r);
  var denominator = crossProduct(r, s);

  if (numerator === 0 && denominator === 0) {
    if (equalPt(p1, q1) || equalPt(p1, q2) || equalPt(p2, q1) || equalPt(p2, q2)) {
      return true;
    }

    if (
      ((q1[0] - p1[0] < 0 === q1[0] - p2[0] < 0) === q2[0] - p1[0] < 0) ===
        q2[0] - p2[0] < 0 &&
      ((q1[1] - p1[1] < 0 === q1[1] - p2[1] < 0) === q2[1] - p1[1] < 0) === q2[1] - p2[1] < 0
    ) {
      return false;
    }

    return true;
  }

  if (denominator === 0) {
    return false;
  }

  var u = numerator / denominator;
  var t = crossProduct(subtractV(q1, p1), s) / denominator;

  return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

function sqDist(a, b) {
  var d0 = a[0] - b[0];
  var d1 = a[1] - b[1];
  return d0 * d0 + d1 * d1;
}

function addV(a, b) {
  return [a[0] + b[0], a[1] + b[1]];
}

function subtractV(a, b) {
  return [a[0] - b[0], a[1] - b[1]];
}

function multV(a, k) {
  return [a[0] * k, a[1] * k];
}

function dotProduct(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

function crossProduct(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

function equalPt(pt1, pt2) {
  return pt1[0] === pt2[0] && pt1[1] === pt2[1];
}

export default {
  /* Hit Test */
  junction: hitTestJunction,
  path: hitTestPath,
  rect: hitTestRect,
  point: hitTestPoint,
  regionPerimeter: hitTestRegionPerimeter,
  lineSegmentsIntersect: hitTestLineSegmentsIntersect
};
