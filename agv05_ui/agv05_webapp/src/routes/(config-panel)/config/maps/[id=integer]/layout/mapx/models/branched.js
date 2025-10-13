/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import Junction from './junction';
import Path from './path';

function getBranchedObject(junctions, paths, stations, ob) {
  if (ob instanceof Junction) {
    return getBranchedObjectForJunction(junctions, paths, stations, ob);
  } else if (ob instanceof Path) {
    return getBranchedObjectForPath(junctions, paths, stations, ob);
  }
  return {};
}

function getBranchedObjectForJunction(junctions, paths, stations, ob) {
  let branchedObject = {};

  let jid = junctions.indexOf(ob);
  if (jid < 0) {
    return branchedObject;
  }
  for (let idx in paths) {
    let path = paths[idx];
    if (!path.tracked) {
      continue;
    }
    if (path.j1 === jid && path.bj1 >= 0) {
      branchedObject[idx] = 0b01;
    }
    if (path.j2 === jid && path.bj2 >= 0) {
      if (branchedObject[idx] === undefined) {
        branchedObject[idx] = 0b00;
      }
      branchedObject[idx] |= 0b10;
    }
  }

  return branchedObject;
}

function getBranchedObjectForPath(junctions, paths, stations, ob) {
  let branchedObject = {};

  let pid = paths.indexOf(ob);
  if (pid < 0 || !ob.tracked) {
    return branchedObject;
  }

  if (ob.bj1 >= 0) {
    branchedObject[pid] = 0b01;
    let branchP = paths[ob.bj1];
    if (branchP && branchP.bj1 === pid) {
      branchedObject[ob.bj1] = 0b01;
    } else if (branchP && branchP.bj2 === pid) {
      branchedObject[ob.bj1] = 0b10;
    }
  }

  if (ob.bj2 >= 0) {
    if (branchedObject[pid] === undefined) {
      branchedObject[pid] = 0b00;
    }
    branchedObject[pid] |= 0b10;
    let branchP = paths[ob.bj2];
    if (branchP && branchP.bj1 === pid) {
      branchedObject[ob.bj2] = 0b01;
    } else if (branchP && branchP.bj2 === pid) {
      branchedObject[ob.bj2] = 0b10;
    }
  }

  return branchedObject;
}

export default {
  /* Aligned */
  get: getBranchedObject
};
