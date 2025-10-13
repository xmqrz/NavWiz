/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

import Junction from '../models/junction';
import NoRotateZone from '../models/no-rotate-zone';
import LocHintZone from '../models/loc-hint-zone';
import Path from '../models/path';
import Station from '../models/station';

export default function (viz, scene, triggerActiveObject) {
  var $viz = $(viz.node());
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

  function containActiveObject(ob) {
    if (!obs || !ob || obs.length === 0) {
      return false;
    }
    return obs.indexOf(ob) >= 0;
  }

  function setActiveObjects(obsNew, focus) {
    if (!Array.isArray(obsNew)) {
      obsNew = obsNew ? [obsNew] : [];
    }

    if (obsNew.length === obs.length && obsNew.every((ob) => obs.indexOf(ob) >= 0)) {
      return;
    }

    if (triggerActiveObject) {
      $viz.trigger('scene.preActiveObject');
    }
    obs = obsNew;
    viz.modelsUpdated();
    if (triggerActiveObject) {
      $viz.trigger('scene.activeObject');

      if (focus && obs.length === 1) {
        let j;
        if (obs[0] instanceof Station) {
          let ob = obs[0];
          let junctions = viz.models.rawJunctions();
          j = junctions[ob.j];
        } else if (obs[0] instanceof Junction) {
          j = obs[0];
        }
        if (j) {
          viz.zoom.zoomCenter(j.x, j.y);
        }
      }
    }
  }

  function addActiveObjects(obsNew) {
    if (!Array.isArray(obsNew)) {
      obsNew = obsNew ? [obsNew] : [];
    }

    if (obsNew.every((ob) => obs.indexOf(ob) >= 0)) {
      return;
    }

    if (triggerActiveObject) {
      $viz.trigger('scene.preActiveObject');
    }

    obsNew.forEach(function (ob) {
      if (obs.indexOf(ob) < 0) {
        obs.push(ob);
      }
    });

    viz.modelsUpdated();
    if (triggerActiveObject) {
      $viz.trigger('scene.activeObject');
    }
  }

  function removeActiveObjects(obsNew) {
    if (!Array.isArray(obsNew)) {
      obsNew = obsNew ? [obsNew] : [];
    }

    if (obsNew.every((ob) => obs.indexOf(ob) < 0)) {
      return;
    }

    if (triggerActiveObject) {
      $viz.trigger('scene.preActiveObject');
    }

    obs = obs.filter((ob) => obsNew.indexOf(ob) < 0);

    viz.modelsUpdated();
    if (triggerActiveObject) {
      $viz.trigger('scene.activeObject');
    }
  }

  function remove() {
    if (!isRemovalAllowed()) {
      return;
    }
    viz.models.push('removeObjects');

    obs.forEach(function (ob) {
      if (ob instanceof Station) {
        viz.models.removeStation(ob);
      } else if (ob instanceof NoRotateZone) {
        viz.models.removeNoRotateZone(ob);
      } else if (ob instanceof LocHintZone) {
        viz.models.removeLocHintZone(ob);
      }
    });

    obs.forEach(function (ob) {
      if (ob instanceof Path) {
        viz.models.removePath(ob);
      }
    });

    setActiveObjects(null);
  }

  function isRemovalAllowed() {
    return (
      obs.every(function (ob) {
        if (ob instanceof Path) {
          return !!viz.models.isPathRemovalAllowed(ob, obs);
        }
        return true;
      }) && obs.some((ob) => !(ob instanceof Junction))
    );
  }

  function setJunctionRfid(rfid) {
    viz.models.push('setJunctionRfid');
    obs.forEach(function (ob) {
      if (!(ob instanceof Junction)) {
        return;
      }
      ob.rfid = rfid;
      viz.models.updateJunction(ob);
    });
  }

  function togglePathFlow() {
    viz.models.push('togglePathFlow');

    var firstOb;
    var firstFlip = false;
    obs.forEach(function (ob) {
      if (!(ob instanceof Path)) {
        return;
      }

      if (firstOb) {
        ob.flow = firstOb.flow;
        if (firstFlip) {
          flipDirection(ob);
        }
        viz.models.updatePath(ob);
        return;
      }

      firstOb = ob;

      if (ob.flow === Path.Flow.BI_DIRECTIONAL) {
        ob.flow = Path.Flow.UNI_DIRECTIONAL;
        if (ob.j1 > ob.j2) {
          firstFlip = true;
          flipDirection(ob);
        }
      } else if (ob.j1 < ob.j2) {
        firstFlip = true;
        flipDirection(ob);
      } else {
        ob.flow = Path.Flow.BI_DIRECTIONAL;
      }

      viz.models.updatePath(ob);
    });

    function flipDirection(ob) {
      [ob.j1, ob.j2] = [ob.j2, ob.j1];
      [ob.bj1, ob.bj2] = [ob.bj2, ob.bj1];

      if (ob.shape === Path.Shape.BEND_LEFT) {
        ob.direction = (ob.direction + 1) % 4;
        ob.shape = Path.Shape.BEND_RIGHT;
      } else if (ob.shape === Path.Shape.BEND_RIGHT) {
        ob.direction = (ob.direction + 3) % 4;
        ob.shape = Path.Shape.BEND_LEFT;
      } else {
        ob.direction = (ob.direction + 2) % 4;
      }
    }
  }

  function togglePathFacing() {
    viz.models.push('togglePathFacing');

    var firstOb;
    for (let ob of obs) {
      if (!(ob instanceof Path)) {
        continue;
      }
      if (ob.flow === Path.Flow.BI_DIRECTIONAL) {
        firstOb = ob;
        break;
      } else if (!firstOb) {
        firstOb = ob;
      }
    }

    if (!firstOb) {
      return;
    }

    var facing = firstOb.facing + 1;
    if (
      facing > Path.Facing.REVERSE_BI ||
      (firstOb.flow === Path.Flow.UNI_DIRECTIONAL && facing === Path.Facing.FORWARD_BI)
    ) {
      facing = 0;
    }

    obs.forEach(function (ob) {
      if (!(ob instanceof Path)) {
        return;
      }
      ob.facing = facing;
      viz.models.updatePath(ob);
    });
  }

  function togglePathBend() {
    var ob = getSingleActiveObject();
    if (!(ob instanceof Path)) {
      return;
    }

    var dataJ = viz.models.rawJunctions();
    if (ob.j1 < 0 || ob.j1 >= dataJ.length || ob.j2 < 0 || ob.j2 >= dataJ.length) {
      console.log("Path's junction index out of bounds.");
      return;
    }
    var j1 = dataJ[ob.j1];
    var j2 = dataJ[ob.j2];
    var oldPath = new Path(ob);

    // Try a different direction.
    if (ob.shape === Path.Shape.STRAIGHT) {
      return;
    } else if (ob.shape === Path.Shape.S_CURVE) {
      if (Path.isNS(ob)) {
        if (j1.x < j2.x) {
          ob.direction = Path.Direction.EAST;
        } else {
          ob.direction = Path.Direction.WEST;
        }
      } else if (Path.isEW(ob)) {
        if (j1.y > j2.y) {
          ob.direction = Path.Direction.NORTH;
        } else {
          ob.direction = Path.Direction.SOUTH;
        }
      } else {
        return;
      }
    } else {
      if (Path.isNS(ob)) {
        if (j1.x < j2.x) {
          ob.direction = Path.Direction.EAST;
          ob.shape = j1.y < j2.y ? Path.Shape.BEND_RIGHT : Path.Shape.BEND_LEFT;
        } else {
          ob.direction = Path.Direction.WEST;
          ob.shape = j1.y < j2.y ? Path.Shape.BEND_LEFT : Path.Shape.BEND_RIGHT;
        }
      } else if (Path.isEW(ob)) {
        if (j1.y > j2.y) {
          ob.direction = Path.Direction.NORTH;
          ob.shape = j1.x > j2.x ? Path.Shape.BEND_LEFT : Path.Shape.BEND_RIGHT;
        } else {
          ob.direction = Path.Direction.SOUTH;
          ob.shape = j1.x > j2.x ? Path.Shape.BEND_RIGHT : Path.Shape.BEND_LEFT;
        }
      } else {
        return;
      }
    }

    if (viz.models.hitTestPath(j1, j2, ob)) {
      Object.assign(ob, oldPath);
    } else {
      var newPath = new Path(ob);
      Object.assign(ob, oldPath);
      viz.models.push('togglePathBend');
      Object.assign(ob, newPath);
      viz.models.updatePath(ob);
    }
  }

  function setPathSpeed(speed) {
    viz.models.push('setPathSpeed');
    obs.forEach(function (ob) {
      if (!(ob instanceof Path)) {
        return;
      }
      ob.speed = speed;
      viz.models.updatePath(ob);
    });
  }

  function getPathSpeed() {
    // return path's common speed or null if path speed not common, otherwise return undefined if no path.
    let speed;
    for (let i = 0; i < obs.length; i++) {
      let ob = obs[i];
      if (!(ob instanceof Path)) {
        continue;
      }

      if (!speed) {
        speed = ob.speed;
        continue;
      }

      if (speed !== ob.speed) {
        return null;
      }
    }

    return speed;
  }

  function setStationName(name) {
    viz.models.push('setStationName');
    var ob = getSingleActiveObject();
    if (!(ob instanceof Station)) {
      return;
    }
    ob.name = name;
    viz.models.updateStation(ob);
  }

  function toggleStationDirection() {
    var movedStations = [];

    obs.forEach(function (ob) {
      if (!(ob instanceof Station)) {
        return;
      }

      var stations = viz.models.lookupStationsFromJunction(ob.j);
      var unmovedStations = stations.filter((s) => s === ob || obs.indexOf(s) < 0);
      if (Array.isArray(stations) && unmovedStations.length <= Station.Direction.NA) {
        var done = false;
        var direction = ob.direction + 1;
        while (!done) {
          if (direction > Station.Direction.NA) {
            direction = 0;
          }

          if (
            !unmovedStations.length ||
            direction > unmovedStations[unmovedStations.length - 1].direction
          ) {
            done = true;
            break;
          }
          for (var i = 0; i < unmovedStations.length; i++) {
            if (direction < unmovedStations[i].direction) {
              done = true;
              break;
            } else if (direction === unmovedStations[i].direction) {
              direction += 1;
            }
          }
        }
        movedStations.push([ob, direction]);
      }
    });

    if (!movedStations.length) {
      return;
    }

    viz.models.push('toggleStationDirection');
    movedStations.forEach(function (d) {
      let [ob, direction] = d;
      ob.direction = direction;
      viz.models.updateStation(ob);
    });
  }

  function nextOverlappedStation() {
    var ob = getSingleActiveObject();
    if (!(ob instanceof Station)) {
      return;
    }
    var stations = viz.models.lookupStationsFromJunction(ob.j);
    if (Array.isArray(stations) && stations.length > 1) {
      for (var i = 0; i < stations.length; i++) {
        if (ob.direction < stations[i].direction) {
          setActiveObjects(stations[i]);
          return;
        }
      }
      setActiveObjects(stations[0]);
    }
  }

  function setLocHintZoneName(name) {
    viz.models.push('setLocHintZoneName');
    var ob = getSingleActiveObject();
    if (!(ob instanceof LocHintZone)) {
      return;
    }
    ob.name = name;
    viz.models.updateLocHintZone(ob);
  }

  return {
    getAll: getActiveObjects,
    contain: containActiveObject,
    get: getSingleActiveObject,
    set: setActiveObjects,
    add: addActiveObjects,
    discard: removeActiveObjects,
    remove: remove,
    isRemovalAllowed: isRemovalAllowed,
    /* junction */
    setJunctionRfid: setJunctionRfid,
    /* path */
    togglePathFlow: togglePathFlow,
    togglePathFacing: togglePathFacing,
    togglePathBend: togglePathBend,
    setPathSpeed: setPathSpeed,
    getPathSpeed: getPathSpeed,
    /* station */
    setStationName: setStationName,
    toggleStationDirection: toggleStationDirection,
    nextOverlappedStation: nextOverlappedStation,
    /* loc hint zone */
    setLocHintZoneName: setLocHintZoneName
  };
}
