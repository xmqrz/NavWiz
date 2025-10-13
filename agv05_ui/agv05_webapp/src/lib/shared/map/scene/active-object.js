/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

export default function (viz, scene, triggerActiveObject) {
  var obs = [];

  function getActiveObjects() {
    return obs;
  }

  function getSingleActiveObject() {
    if (!obs || obs.length !== 1) {
      return null;
    }
    return obs[0];
  }

  function setActiveObject(obsNew) {
    if (!Array.isArray(obsNew)) {
      obsNew = obsNew ? [obsNew] : [];
    }

    if (obsNew.length === obs.length && obsNew.every((ob) => obs.indexOf(ob) >= 0)) {
      return;
    }

    obs = obsNew;
    viz.modelsUpdated();
  }

  return {
    getAll: getActiveObjects,
    get: getSingleActiveObject,
    set: setActiveObject
  };
}
