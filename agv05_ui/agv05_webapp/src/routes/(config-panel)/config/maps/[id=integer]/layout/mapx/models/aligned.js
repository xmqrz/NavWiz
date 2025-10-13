/*
 * Copyright (c) 2019, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import Junction from './junction';
import Path from './path';
import Station from './station';

const alignThresholdSq = Math.pow(Math.sin((5 * Math.PI) / 180), 2);
const vectorTable = [];
for (let i = 0; i < 360; i += 10) {
  vectorTable.push([Math.cos((i * Math.PI) / 180), Math.sin((i * Math.PI) / 180)]);
}
const emptyObjects = {
  paths: {},
  stations: {},
  ob: 0
};

function getAlignedObjects(junctions, paths, stations, ob) {
  if (ob instanceof Junction) {
    return getAlignedObjectsForJunction(junctions, paths, stations, ob);
  } else if (ob instanceof Path) {
    if (ob.shape === Path.Shape.STRAIGHT) {
      return getAlignedObjectsForStraightPath(junctions, paths, stations, ob);
    } else if (ob.shape === Path.Shape.BEZIER) {
      return getAlignedObjectsForBezierPath(junctions, paths, stations, ob);
    }
  } else if (ob instanceof Station) {
    return getAlignedObjectsForStation(junctions, paths, stations, ob);
  }
  return emptyObjects;
}

/* Helper functions */
function getAlignedObjectsForJunction(_junctions, _paths, _stations, _ob) {
  return emptyObjects;
}

function getAlignedObjectsForStraightPath(junctions, paths, stations, ob) {
  var alignedPaths = {};
  var alignedStations = {};
  var alignJ1 = false;
  var alignJ2 = false;

  var v = getPathVector(junctions, ob);
  var dot = dotProduct(v, v);

  paths.forEach(function (p, idx) {
    if (ob === p) {
      return;
    }
    let [e11, e12, e21, e22] = [
      ob.j1 === p.j1,
      ob.j1 === p.j2,
      ob.j2 === p.j1,
      ob.j2 === p.j2
    ];
    if (p.shape === Path.Shape.STRAIGHT) {
      let [e1, e2] = [e11 || e12, e21 || e22];
      if (e1 || e2) {
        let pv = getPathVector(junctions, p);
        if (isVectorsAligned(v, dot, pv)) {
          alignedPaths[idx] = (e11 || e21) | ((e12 || e22) << 1);
          alignJ1 |= e1;
          alignJ2 |= e2;
        }
      }
    } else if (p.shape === Path.Shape.BEZIER) {
      if (e11 || e21) {
        let pv = getPathCP1Vector(junctions, p);
        if (isVectorsAligned(v, dot, pv)) {
          alignedPaths[idx] = 1;
          alignJ1 |= e11;
          alignJ2 |= e21;
        }
      }
      if (e12 || e22) {
        let pv = getPathCP2Vector(junctions, p);
        if (isVectorsAligned(v, dot, pv)) {
          alignedPaths[idx] |= 2;
          alignJ1 |= e12;
          alignJ2 |= e22;
        }
      }
    }
  });
  stations.forEach(function (st, idx) {
    let [e1, e2] = [ob.j1 === st.j, ob.j2 === st.j];
    if (e1 || e2) {
      let sv = getStationVector(st);
      if (sv && isVectorsAligned(v, dot, sv)) {
        alignedStations[idx] = 1;
        alignJ1 |= e1;
        alignJ2 |= e2;
      }
    }
  });
  return {
    paths: alignedPaths,
    stations: alignedStations,
    ob: alignJ1 | (alignJ2 << 1)
  };
}

function getAlignedObjectsForBezierPath(junctions, paths, stations, ob) {
  var alignedPaths = {};
  var alignedStations = {};
  var alignJ1 = false;
  var alignJ2 = false;

  var v1 = getPathCP1Vector(junctions, ob);
  var v2 = getPathCP2Vector(junctions, ob);
  var dot1 = dotProduct(v1, v1);
  var dot2 = dotProduct(v2, v2);

  paths.forEach(function (p, idx) {
    if (ob === p) {
      return;
    }
    let [e11, e12, e21, e22] = [
      ob.j1 === p.j1,
      ob.j1 === p.j2,
      ob.j2 === p.j1,
      ob.j2 === p.j2
    ];
    if (p.shape === Path.Shape.STRAIGHT) {
      let [e1, e2] = [e11 || e12, e21 || e22];
      if (e1 || e2) {
        let pv = getPathVector(junctions, p);
        if (e1 && isVectorsAligned(v1, dot1, pv)) {
          alignedPaths[idx] = e11 | (e12 << 1);
          alignJ1 = true;
        }
        if (e2 && isVectorsAligned(v2, dot2, pv)) {
          alignedPaths[idx] |= e21 | (e22 << 1);
          alignJ2 = true;
        }
      }
    } else if (p.shape === Path.Shape.BEZIER) {
      if (e11 || e12) {
        let pv = (e11 ? getPathCP1Vector : getPathCP2Vector)(junctions, p);
        if (isVectorsAligned(v1, dot1, pv)) {
          alignedPaths[idx] = e11 | (e12 << 1);
          alignJ1 = true;
        }
      }
      if (e21 || e22) {
        let pv = (e21 ? getPathCP1Vector : getPathCP2Vector)(junctions, p);
        if (isVectorsAligned(v2, dot2, pv)) {
          alignedPaths[idx] |= e21 | (e22 << 1);
          alignJ2 = true;
        }
      }
    }
  });
  stations.forEach(function (st, idx) {
    if (ob.j1 === st.j) {
      let sv = getStationVector(st);
      if (sv && isVectorsAligned(v1, dot1, sv)) {
        alignedStations[idx] = 1;
        alignJ1 = true;
      }
    } else if (ob.j2 === st.j) {
      let sv = getStationVector(st);
      if (sv && isVectorsAligned(v2, dot2, sv)) {
        alignedStations[idx] = 1;
        alignJ2 = true;
      }
    }
  });
  return {
    paths: alignedPaths,
    stations: alignedStations,
    ob: alignJ1 | (alignJ2 << 1)
  };
}

function getAlignedObjectsForStation(junctions, paths, stations, ob) {
  var alignedPaths = {};
  var align = false;

  var v = getStationVector(ob);
  if (!v) {
    return emptyObjects;
  }
  paths.forEach(function (p, idx) {
    let pv;
    let [e1, e2] = [ob.j === p.j1, ob.j === p.j2];
    if (p.shape === Path.Shape.STRAIGHT) {
      if (e1 || e2) {
        pv = getPathVector(junctions, p);
      }
    } else if (p.shape === Path.Shape.BEZIER) {
      if (e1) {
        pv = getPathCP1Vector(junctions, p);
      } else if (e2) {
        pv = getPathCP2Vector(junctions, p);
      }
    }
    if (pv && isVectorsAligned(v, 1, pv)) {
      alignedPaths[idx] = e1 | (e2 << 1);
      align = true;
    }
  });
  return {
    paths: alignedPaths,
    stations: {},
    ob: align
  };
}

function getPathVector(junctions, p) {
  var j1 = junctions[p.j1];
  var j2 = junctions[p.j2];
  return [j2.x - j1.x, j2.y - j1.y];
}

function getPathCP1Vector(junctions, p) {
  var j1 = junctions[p.j1];
  return [p.cp1.x - j1.x, p.cp1.y - j1.y];
}

function getPathCP2Vector(junctions, p) {
  var j2 = junctions[p.j2];
  return [p.cp2.x - j2.x, p.cp2.y - j2.y];
}

function getStationVector(st) {
  return vectorTable[Math.round(st.heading / 10)];
}

function isVectorsAligned(a, aDot, b) {
  var bDot = dotProduct(b, b);
  var cross = crossProduct(a, b);
  return cross * cross <= aDot * bDot * alignThresholdSq;
}

function dotProduct(a, b) {
  return a[0] * b[0] + a[1] * b[1];
}

function crossProduct(a, b) {
  return a[0] * b[1] - a[1] * b[0];
}

export default {
  /* Aligned */
  get: getAlignedObjects
};
