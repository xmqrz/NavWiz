/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import $ from 'cash-dom';
import TextAnnotation from '../../map_annotation/models/text-annotation';
import PolygonAnnotation from '../../map_annotation/models/polygon-annotation';
import IconAnnotation from '../../map_annotation/models/icon-annotation';

export default function (viz, scene, triggerActiveObject) {
  var $viz = $(viz.node());
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

  function setActiveObjects(obsNew, focus = null) {
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
    viz.models.push('removeObjects');

    obs.forEach(function (ob) {
      if (ob instanceof TextAnnotation) {
        viz.models.removeTextAnnotation(ob);
      } else if (ob instanceof PolygonAnnotation) {
        viz.models.removePolygonAnnotation(ob);
      } else if (ob instanceof IconAnnotation) {
        viz.models.removeIconAnnotation(ob);
      }
    });

    setActiveObjects(null);
  }

  /* text annotation */
  function setFontSize(size) {
    viz.models.push('setFontSize');
    obs.forEach(function (ob) {
      if (!(ob instanceof TextAnnotation)) {
        return;
      }
      ob.size = size;
      viz.models.updateTextAnnotation(ob);
    });
  }

  function setTextAnnotationContent(content) {
    viz.models.push('setTextAnnotationContent');
    obs.forEach(function (ob) {
      if (!(ob instanceof TextAnnotation)) {
        return;
      }
      if (content === '' || content.trim() === '') {
        content = 'Add text'; // reset the content to default
      }
      ob.content = content;
      viz.models.updateTextAnnotation(ob);
    });
  }

  /* polygon annotation */
  function setPolygonFillColor(color) {
    viz.models.push('setFillColor');
    obs.forEach(function (ob) {
      if (!(ob instanceof PolygonAnnotation)) {
        return;
      }
      ob.fill = color;
      viz.models.updatePolygonAnnotation(ob);
    });
  }

  function setPolygonFillOpacity(opacity) {
    viz.models.push('setFillOpacity');
    obs.forEach(function (ob) {
      if (!(ob instanceof PolygonAnnotation)) {
        return;
      }
      ob.fillOpacity = opacity;
      viz.models.updatePolygonAnnotation(ob);
    });
  }

  return {
    getAll: getActiveObjects,
    contain: containActiveObject,
    get: getSingleActiveObject,
    set: setActiveObjects,
    add: addActiveObjects,
    discard: removeActiveObjects,
    remove: remove,
    /* text annotation */
    setFontSize: setFontSize,
    setTextAnnotationContent: setTextAnnotationContent,
    /* polygon annotation */
    setPolygonFillColor: setPolygonFillColor,
    setPolygonFillOpacity: setPolygonFillOpacity,
    // placeholder for map rendering
    getAligned: () => aligned,
    getBranched: () => {
      return {};
    }
  };
}
