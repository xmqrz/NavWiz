/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

import ForbiddenZone from '../models/forbidden-zone';
import Junction from '../models/junction';
import Landmark from '../models/landmark';
import NoRotateZone from 'map-layout-editor/models/no-rotate-zone';
import LocHintZone from 'map-layout-editor/models/loc-hint-zone';
import Path from '../models/path';
import Station from '../models/station';

export default function (viz, scene, triggerActiveObject) {
  var $viz = $(viz.node());
  // var Utils = scene.Utils;
  var obs = [];
  var aligned = {
    paths: {},
    stations: {},
    ob: false
  };

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
    if (obs.length === 1) {
      aligned = viz.models.getAligned(obs[0]);
    } else {
      aligned = viz.models.getAligned();
    }
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

    if (obs.length === 1) {
      aligned = viz.models.getAligned(obs[0]);
    } else {
      aligned = viz.models.getAligned();
    }

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

    if (obs.length === 1) {
      aligned = viz.models.getAligned(obs[0]);
    } else {
      aligned = viz.models.getAligned();
    }

    viz.modelsUpdated();
    if (triggerActiveObject) {
      $viz.trigger('scene.activeObject');
    }
  }

  function expandActiveObject() {
    var visibleLayers = viz.models.rawVisibleLayers();
    let expandedSelections = [];
    let selectedJunctions = [];
    let paths = viz.models.rawPaths();
    let junctions = viz.models.rawJunctions();
    let stations = viz.models.rawStations();
    for (let o of obs) {
      if (o instanceof Junction) {
        selectedJunctions.push(o);
      } else if (o instanceof Path) {
        if (o.j1 < 0 || o.j1 >= junctions.length || o.j2 < 0 || o.j2 >= junctions.length) {
          console.log("Path's junction index out of bounds.");
          continue;
        }
        expandedSelections.push(junctions[o.j1]);
        expandedSelections.push(junctions[o.j2]);
      } else if (o instanceof Station) {
        if (o.j < 0 || o.j >= junctions.length) {
          console.log("Station's junction index out of bounds.");
          continue;
        }
        expandedSelections.push(junctions[o.j]);
      }
    }
    for (let j of selectedJunctions) {
      let jIdx = junctions.indexOf(j);
      for (let p of paths) {
        if (p.j1 === jIdx || p.j2 === jIdx) {
          expandedSelections.push(p);
        }
      }
      for (let s of stations) {
        if (s.j === jIdx) {
          expandedSelections.push(s);
        }
      }
    }
    addActiveObjects(expandedSelections.filter((o) => visibleLayers[o.layer]));
  }

  function remove() {
    if (!isRemovalAllowed()) {
      return;
    }
    viz.models.push('removeObjects');
    aligned = viz.models.getAligned();

    obs.forEach(function (ob) {
      if (ob instanceof Station) {
        viz.models.removeStation(ob);
      } else if (ob instanceof Landmark) {
        viz.models.removeLandmark(ob);
      } else if (ob instanceof ForbiddenZone) {
        viz.models.removeForbiddenZone(ob);
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

  function getBranched() {
    var ob = getSingleActiveObject();
    return viz.models.getBranched(ob);
  }

  function setLayer(layer) {
    if (obs.length <= 0) {
      return;
    }
    viz.models.push('setLayer');
    obs.forEach(function (ob) {
      if (ob instanceof Junction) {
        ob.layer = layer;
        viz.models.updateJunction(ob);
      } else if (ob instanceof Path) {
        ob.layer = layer;
        viz.models.updatePath(ob);
      } else if (ob instanceof Station) {
        ob.layer = layer;
        viz.models.updateStation(ob);
      }
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

      aligned = viz.models.getAligned(ob);
      viz.models.updatePath(ob);
    });

    function flipDirection(ob) {
      [ob.j1, ob.j2] = [ob.j2, ob.j1];
      [ob.cp1, ob.cp2] = [ob.cp2, ob.cp1];
      [ob.bj1, ob.bj2] = [ob.bj2, ob.bj1];
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

  function togglePathTracked() {
    viz.models.push('togglePathTracked');

    var firstOb;
    obs.forEach(function (ob) {
      if (!(ob instanceof Path)) {
        return;
      }

      if (firstOb) {
        ob.dynamic = firstOb.dynamic;
        ob.tracked = firstOb.tracked;
        viz.models.updatePath(ob);
        return;
      }

      firstOb = ob;

      if (ob.dynamic) {
        ob.dynamic = false;
        ob.tracked = true;
      } else if (ob.tracked) {
        ob.tracked = false;
      } else {
        if (viz.options.dynamic) {
          ob.dynamic = true;
        } else {
          ob.tracked = true;
        }
      }
      viz.models.updatePath(ob);
    });
  }

  function setPathDistance(distance) {
    var ob = getSingleActiveObject();
    if (!(ob instanceof Path)) {
      return;
    }
    ob.distance = distance;
    aligned = viz.models.getAligned(ob);
    viz.models.updatePath(ob);
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

  function toggleStationHeading(leap) {
    var movedStations = [];
    leap = leap === true;

    obs.forEach(function (ob) {
      if (!(ob instanceof Station)) {
        return;
      }

      var stations = viz.models.lookupStationsFromJunction(ob.j);
      var unmovedStations = stations.filter((s) => s === ob || obs.indexOf(s) < 0);
      if (Array.isArray(stations) && unmovedStations.length <= Station.Heading.SEGMENTS) {
        var done = false;
        var heading =
          ob.heading + (leap ? Station.Heading.QUARTER : Station.Heading.RESOLUTION);
        while (!done) {
          if (leap && heading >= Station.Heading.NA) {
            heading -= Station.Heading.NA;
          } else if (heading > Station.Heading.NA) {
            heading = 0;
          }
          if (
            !unmovedStations.length ||
            heading > unmovedStations[unmovedStations.length - 1].heading
          ) {
            done = true;
            break;
          }
          for (var i = 0; i < unmovedStations.length; i++) {
            if (heading < unmovedStations[i].heading) {
              done = true;
              break;
            } else if (heading === unmovedStations[i].heading) {
              heading += Station.Heading.RESOLUTION;
            }
          }
        }
        movedStations.push([ob, heading]);
      }
    });

    if (!movedStations.length) {
      return;
    }

    viz.models.push('toggleStationHeading');
    movedStations.forEach(function (d) {
      let [ob, heading] = d;
      ob.heading = heading;
      viz.models.updateStation(ob);
    });

    let ob = getSingleActiveObject();
    aligned = viz.models.getAligned(ob);
    viz.modelsUpdated();
  }

  function toggleStationHeadingReversed(leap) {
    var movedStations = [];
    leap = leap === true;

    obs.forEach(function (ob) {
      if (!(ob instanceof Station)) {
        return;
      }

      var stations = viz.models.lookupStationsFromJunction(ob.j);
      var unmovedStations = stations.filter((s) => s === ob || obs.indexOf(s) < 0);
      if (Array.isArray(stations) && unmovedStations.length <= Station.Heading.SEGMENTS) {
        var done = false;
        var heading =
          ob.heading - (leap ? Station.Heading.QUARTER : Station.Heading.RESOLUTION);
        while (!done) {
          if (heading < 0) {
            heading = leap ? heading + Station.Heading.NA : Station.Heading.NA;
          }
          if (!unmovedStations.length || heading < unmovedStations[0].heading) {
            done = true;
            break;
          }
          for (var i = unmovedStations.length - 1; i >= 0; i--) {
            if (heading > unmovedStations[i].heading) {
              done = true;
              break;
            } else if (heading === unmovedStations[i].heading) {
              heading -= Station.Heading.RESOLUTION;
            }
          }
        }
        movedStations.push([ob, heading]);
      }
    });

    if (!movedStations.length) {
      return;
    }

    viz.models.push('toggleStationHeadingReversed');
    movedStations.forEach(function (d) {
      let [ob, heading] = d;
      ob.heading = heading;
      viz.models.updateStation(ob);
    });

    let ob = getSingleActiveObject();
    aligned = viz.models.getAligned(ob);
    viz.modelsUpdated();
  }

  function nextOverlappedStation() {
    var ob = getSingleActiveObject();
    if (!(ob instanceof Station)) {
      return;
    }
    var stations = viz.models.lookupStationsFromJunction(ob.j);
    if (Array.isArray(stations) && stations.length > 1) {
      for (var i = 0; i < stations.length; i++) {
        if (ob.heading < stations[i].heading) {
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
    expand: expandActiveObject,
    remove: remove,
    isRemovalAllowed: isRemovalAllowed,
    getAligned: () => aligned,
    getBranched: getBranched,
    setLayer: setLayer,
    /* path */
    togglePathFlow: togglePathFlow,
    togglePathFacing: togglePathFacing,
    togglePathTracked: togglePathTracked,
    setPathDistance: setPathDistance,
    setPathSpeed: setPathSpeed,
    getPathSpeed: getPathSpeed,
    /* station */
    setStationName: setStationName,
    toggleStationHeading: toggleStationHeading,
    toggleStationHeadingReversed: toggleStationHeadingReversed,
    nextOverlappedStation: nextOverlappedStation,
    /* loc hint zone */
    setLocHintZoneName: setLocHintZoneName
  };
}
