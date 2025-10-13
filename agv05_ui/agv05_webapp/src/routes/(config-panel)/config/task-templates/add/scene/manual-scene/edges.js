/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $perf from '$lib/shared/perf';
import { AvoidLib } from 'libavoid-js';
import avoidLibWASM from 'node_modules/libavoid-js/dist/libavoid.wasm?url';

// TODO: update using async for load.
var Avoid;
var avoidModelsUpdated;
AvoidLib.load(avoidLibWASM).then(function () {
  Avoid = AvoidLib.getInstance();
  if (avoidModelsUpdated !== undefined) {
    avoidModelsUpdated[0](avoidModelsUpdated[1], avoidModelsUpdated[2]);
  }
});

export default function (manualScene, scene) {
  /* Edges */
  var edges = scene.append('g');
  edges.attr('class', 'edges');

  function modelsUpdated(edgesData, nodesData) {
    if (!Avoid) {
      avoidModelsUpdated = [modelsUpdated, edgesData, nodesData];
      return;
    }

    edges.selectAll('g.edge').remove();

    if (edgesData.length <= 0) {
      return;
    }

    edgesData = edgesData.sort((a, b) => (a.active === b.active ? 0 : a.active ? -1 : 1));

    generateEdge(edgesData, nodesData);
  }

  function generateEdge(edgesData, nodesData) {
    $perf.begin('task-template.edge-layout-viz');
    let router = new Avoid.Router(Avoid.PolyLineRouting);

    router.setRoutingOption(Avoid.nudgeSharedPathsWithCommonEndPoint, true);
    router.setRoutingOption(
      Avoid.improveHyperedgeRoutesMovingAddingAndDeletingJunctions,
      true
    );
    router.setRoutingParameter(Avoid.idealNudgingDistance, 20);
    router.setRoutingParameter(Avoid.shapeBufferDistance, 5);

    nodesData.forEach(function (node) {
      let shape = new Avoid.Rectangle(
        new Avoid.Point(node.x, node.y),
        node.width + 20,
        node.height
      );
      new Avoid.ShapeRef(router, shape);
    });

    edgesData.forEach(function (edge) {
      let srcPt = new Avoid.Point(edge.srcPos[0], edge.srcPos[1] + 20);
      let dstPt;
      if (edge.loop) {
        dstPt = new Avoid.Point(edge.dstPos[0], edge.dstPos[1] - edge.dstDim[1] / 2 - 10);
      } else {
        dstPt = new Avoid.Point(edge.dstPos[0], edge.dstPos[1]);
      }

      let srcConnEnd = new Avoid.ConnEnd(srcPt);
      let dstConnEnd = new Avoid.ConnEnd(dstPt);
      let connRef = new Avoid.ConnRef(router, srcConnEnd, dstConnEnd);
      connRef.setCallback(() => {
        let route = connRef.displayRoute();
        drawConnector(route, edge);
      }, connRef);
    });

    router.processTransaction();

    $perf.end('task-template.edge-layout-viz');
  }

  function drawConnector(route, edge) {
    let e = edges.append('g');
    e.attr('class', 'edge');

    let path = postProcessPath(route, edge);

    e.append('title').text(edge.title);
    let p = e.append('path');
    p.attr('d', generateConnectorPath(path))
      .attr('stroke', edge.active ? 'red' : 'black')
      .attr('stroke-linejoin', 'round')
      .attr('stroke-width', 1.5)
      .attr('fill', 'none');
    e.append('polygon')
      .attr('fill', edge.drag ? 'ghostwhite' : edge.active ? 'red' : 'black')
      .attr('transform', generateArrowTransform(path, p))
      .attr('points', '0,0 -3.5,-10 3.5,-10 0,0');
  }

  function postProcessPath(route, edge) {
    // cut path up to dst bounding box.
    let path = [[route.get_ps(0).x, route.get_ps(0).y - 20]];

    let minX = edge.dstPos[0] - edge.dstDim[0] / 2;
    let maxX = edge.dstPos[0] + edge.dstDim[0] / 2;
    let minY = edge.dstPos[1] - edge.dstDim[1] / 2;
    let maxY = edge.dstPos[1] + edge.dstDim[1] / 2;

    for (let i = 0, j = 1; i < route.size() - 1; i++, j++) {
      let cur = route.get_ps(i);
      let next = route.get_ps(j);

      path.push([cur.x, cur.y]);

      if (next.x >= minX && next.x <= maxX && next.y >= minY && next.y <= maxY) {
        let intersec = lineBoxIntersect(cur.x, cur.y, next.x, next.y, [
          minX,
          maxX,
          minY,
          maxY
        ]);
        if (intersec !== null) {
          path.push(intersec);
        }
        break;
      }
    }

    if (edge.loop) {
      let cur = path[path.length - 1];
      let last = route.get_ps(route.size() - 1);
      let intersec = lineBoxIntersect(cur[0], cur[1], last.x, last.y + 30, [
        minX,
        maxX,
        minY,
        maxY
      ]);
      path.push(intersec);
    }

    return path;
  }

  function generateConnectorPath(path) {
    let cuts = 3;
    let coords = chaikinPathCornerCutting(path, cuts);
    return `M ${path[0][0]},${path[0][1]} L ${coords.join(' ')}`;
  }

  function chaikinPathCornerCutting(arr, num) {
    if (num <= 0) {
      return arr;
    }
    let smooth = [];
    for (let i = 0; i < arr.length; i++) {
      let c = arr[i];
      if (i === arr.length - 1) {
        smooth.push(c);
        continue;
      }
      smooth.push([0.85 * c[0] + 0.15 * arr[i + 1][0], 0.85 * c[1] + 0.15 * arr[i + 1][1]]);
      smooth.push([0.15 * c[0] + 0.85 * arr[i + 1][0], 0.15 * c[1] + 0.85 * arr[i + 1][1]]);
    }
    return chaikinPathCornerCutting(smooth, num - 1);
  }

  function generateArrowTransform(path, p) {
    let pt = p.node().getPointAtLength(p.node().getTotalLength() - 10);
    let start = [pt.x, pt.y];
    let end = path[path.length - 1];

    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let deg = (Math.atan2(dy, dx) * 180.0) / Math.PI;
    return `translate(${end[0]},${end[1]})rotate(${deg - 90})`;
  }

  // Utils

  function lineBoxIntersect(x0, y0, x1, y1, bbox) {
    var [xmin, xmax, ymin, ymax] = bbox;
    var t0 = 0,
      t1 = 1;
    var dx = x1 - x0,
      dy = y1 - y0;
    var p, q, r;

    for (var edge = 0; edge < 4; edge++) {
      if (edge === 0) {
        p = -dx;
        q = -(xmin - x0);
      }
      if (edge === 1) {
        p = dx;
        q = xmax - x0;
      }
      if (edge === 2) {
        p = -dy;
        q = -(ymin - y0);
      }
      if (edge === 3) {
        p = dy;
        q = ymax - y0;
      }

      r = q / p;

      if (p === 0 && q < 0) {
        return null;
      }

      if (p < 0) {
        if (r > t1) {
          return null;
        } else if (r > t0) {
          t0 = r;
        }
      } else if (p > 0) {
        if (r < t0) {
          return null;
        } else if (r < t1) {
          t1 = r;
        }
      }
    }

    return [x0 + t0 * dx, y0 + t0 * dy];
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
