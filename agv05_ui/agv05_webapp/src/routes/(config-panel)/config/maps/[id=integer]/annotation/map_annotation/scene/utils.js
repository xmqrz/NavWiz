/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import TextAnnotation from '../models/text-annotation';
import PolygonAnnotation from '../models/polygon-annotation';
import IconAnnotation from '../models/icon-annotation';

export default function (viz, _scene) {
  /* Utilities */
  function makeTextAnnotation(pt, ob) {
    /* the object being updated can be supplied as ob, for example when dragging */
    var textAnnotation = new TextAnnotation({
      x: pt[0],
      y: pt[1],
      content: ''
    });
    textAnnotation.ob = ob;

    return textAnnotation;
  }

  function makePolygonAnnotation(pt1, pt2) {
    if (!pt2) {
      return new PolygonAnnotation({
        points: [pt1, pt1, pt1, pt1]
      });
    }
    // generate points of a rectangle in anti-clockwise order
    var dx = pt2[0] - pt1[0];
    var dy = pt2[1] - pt1[1];
    var dxdy = dx * dy;

    if (dxdy === 0) {
      return false;
    } else if (dxdy > 0) {
      return new PolygonAnnotation({
        points: [pt1, [pt2[0], pt1[1]], pt2, [pt1[0], pt2[1]]]
      });
    } else {
      return {
        points: [pt1, [pt1[0], pt2[1]], pt2, [pt2[0], pt1[1]]]
      };
    }
  }

  function makeIconAnnotation(pt, ob) {
    /* the object being updated can be supplied as ob, for example when dragging */
    var icon = new IconAnnotation({
      x: pt[0],
      y: pt[1]
    });
    icon.ob = ob;
    return icon;
  }

  function makeDraggables(pt, dg) {
    /**
     * Updates draggables (dg) and returns the validity of the dragged position
     *
     * pt: new drag position
     * dg: {
     *    ctx:  original selected objects
     *    cp:   original point as control point
     *    mctx: modified selected objects
     * }
     */
    if (!dg || !dg.ctx) {
      return false;
    }
    if (!dg.off) {
      dg.off = dg.ctx.map((ob) => ({
        x: ob.x - dg.cp.x,
        y: ob.y - dg.cp.y
      }));
    }

    dg.mctx = [];
    dg.ctx.map(function (ob, i) {
      let _pt = viz.zoom.computeSnap([pt[0] + dg.off[i].x, pt[1] + dg.off[i].y], 0.05);
      if (ob instanceof TextAnnotation) {
        let ta = new TextAnnotation(ob);
        ta.x = _pt[0];
        ta.y = _pt[1];
        dg.mctx.push(ta);
      } else if (ob instanceof IconAnnotation) {
        let ia = new IconAnnotation(ob);
        ia.x = _pt[0];
        ia.y = _pt[1];
        dg.mctx.push(ia);
      }
    });
    return true;
  }

  function saveDraggables(dg) {
    viz.models.push('dragAnnotations');
    for (let i = 0; i < dg.mctx.length; ++i) {
      let ob = dg.mctx[i];
      dg.ctx[i].x = ob.x;
      dg.ctx[i].y = ob.y;
    }
    viz.models.triggerDirty();
    return dg.ctx;
  }

  function makeClone(pt, cln) {
    /**
     * Updates clone (cln) and returns the validity of the dragged position
     *
     * pt: new drag position
     * dg: {
     *    ctx:  original selected objects
     *    cp:   original point as control point
     *    textAnnotations: new text annotations
     *    iconAnnotations: new icon annotations
     * }
     */
    if (!cln || !cln.ctx) {
      return false;
    }

    if (!cln.off) {
      cln.off = cln.ctx.map((ob) => ({
        x: ob.x - cln.cp.x,
        y: ob.y - cln.cp.y
      }));
    }

    cln.textAnnotations = [];
    cln.iconAnnotations = [];

    cln.ctx.map(function (ob, i) {
      let _pt = viz.zoom.computeSnap([pt[0] + cln.off[i].x, pt[1] + cln.off[i].y], 0.05);
      if (ob instanceof TextAnnotation) {
        let ta = new TextAnnotation(ob);
        ta.x = _pt[0];
        ta.y = _pt[1];
        cln.textAnnotations.push(ta);
      } else if (ob instanceof IconAnnotation) {
        let ia = new IconAnnotation(ob);
        ia.x = _pt[0];
        ia.y = _pt[1];
        cln.iconAnnotations.push(ia);
      }
    });
    return true;
  }

  function saveClone(cln) {
    viz.models.push('cloneAnnotations');

    let cloneTextAnnotations = [];
    let cloneIconAnnotations = [];

    for (let ta of cln.textAnnotations) {
      let i = viz.models.addTextAnnotation(ta, false);
      cloneTextAnnotations.push(viz.models.rawTextAnnotations()[i]);
    }
    for (let ia of cln.iconAnnotations) {
      let i = viz.models.addIconAnnotation(ia, false);
      cloneIconAnnotations.push(viz.models.rawIconAnnotations()[i]);
    }

    viz.models.triggerDirty();
    return cloneTextAnnotations.concat(cloneIconAnnotations);
  }

  function makeSelectionArea(pt1, pt2) {
    if (!pt2) {
      return {
        points: [pt1, pt1, pt1, pt1]
      };
    }
    // generate points of a rectangle in anti-clockwise order
    var dx = pt2[0] - pt1[0];
    var dy = pt2[1] - pt1[1];
    var dxdy = dx * dy;

    if (dxdy > 0) {
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
    /* snap to an existing textAnnotation or iconAnnotation */
    let hit = snapToTextAnnotation(pt);
    return hit ? hit : snapToIconAnnotation(pt);
  }

  function snapToTextAnnotation(pt) {
    /* snap to an existing TextAnnotation. */
    var sd = viz.zoom.getSnapDistance();
    var lsd = viz.zoom.getLowerSnapDistance();
    var hit;

    if (lsd) {
      let _pt = snap(pt, lsd);
      hit = viz.models.hitTestTextAnnotation(
        new TextAnnotation({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    if (!hit) {
      let _pt = snap(pt, sd);
      hit = viz.models.hitTestTextAnnotation(
        new TextAnnotation({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    return hit ? hit : null;
  }

  function snapToIconAnnotation(pt) {
    /* snap to an existing iconAnnotation. */
    var sd = viz.zoom.getSnapDistance();
    var lsd = viz.zoom.getLowerSnapDistance();
    var hit;

    if (lsd) {
      let _pt = snap(pt, lsd);
      hit = viz.models.hitTestIconAnnotation(
        new IconAnnotation({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    if (!hit) {
      let _pt = snap(pt, sd);
      hit = viz.models.hitTestIconAnnotation(
        new IconAnnotation({
          x: _pt[0],
          y: _pt[1]
        })
      );
    }
    return hit ? hit : null;
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

  return {
    makeTextAnnotation: makeTextAnnotation,
    makePolygonAnnotation: makePolygonAnnotation,
    makeIconAnnotation: makeIconAnnotation,
    makeDraggables: makeDraggables,
    saveDraggables: saveDraggables,
    makeClone: makeClone,
    saveClone: saveClone,
    makeSelectionArea: makeSelectionArea,
    objectsInArea: objectsInArea,
    snap: snap,
    snapToObject: snapToObject,
    snapToTextAnnotation: snapToTextAnnotation,
    snapToIconAnnotation: snapToIconAnnotation,
    withinCorners: withinCorners
  };
}
