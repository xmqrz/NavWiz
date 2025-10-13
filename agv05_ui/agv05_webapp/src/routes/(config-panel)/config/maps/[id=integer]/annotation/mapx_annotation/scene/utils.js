/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

import TextAnnotation from '../../map_annotation/models/text-annotation';
import PolygonAnnotation from '../../map_annotation/models/polygon-annotation';
import IconAnnotation from '../../map_annotation/models/icon-annotation';

export default function (viz, scene) {
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
      let _pt = viz.zoom.computeSnap([pt[0] + dg.off[i].x, pt[1] + dg.off[i].y], 0.01);
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
      let _pt = viz.zoom.computeSnap([pt[0] + cln.off[i].x, pt[1] + cln.off[i].y], 0.01);
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
      pt[0] >= corners.bottom &&
      pt[0] <= corners.top &&
      pt[1] >= corners.right &&
      pt[1] <= corners.left
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
    withinCorners: withinCorners,
    generateRulerPath: generateRulerPath
  };
}

/* Helper functions */
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
