import { isBefore, differenceInHours } from 'date-fns';
import YAxis from './y-axis.js';
import XAxis from './x-axis.js';
import Plot from './plot.js';
import Legend from './legend.js';
import Utils from './utils.js';

export default function (viz) {
  let scene = {};

  // TODO: handle if x data is not date object
  const [minX, maxX] = viz.data.plot.reduce(
    (acc, val) => {
      if (acc[0] === undefined || isBefore(val[0][0], acc[0])) {
        acc[0] = val[0][0];
      }
      if (acc[1] === undefined || isBefore(acc[1], val[val.length - 1][0])) {
        acc[1] = val[val.length - 1][0];
      }
      return acc;
    },
    [undefined, undefined]
  );

  const ticksX = differenceInHours(maxX, minX);

  const [minY, maxY] = viz.data.plot.reduce(
    (acc, val) => {
      const [minV, maxV] = val.reduce(
        (accV, v) => {
          if (accV[0] == null || accV[0] > v[1]) {
            accV[0] = v[1];
          }
          if (accV[1] == null || accV[1] < v[1]) {
            accV[1] = v[1];
          }
          return accV;
        },
        [null, null]
      );
      if (acc[0] == null || acc[0] > minV) {
        acc[0] = minV;
      }
      if (acc[1] == null || acc[1] < maxV) {
        acc[1] = maxV;
      }
      return acc;
    },
    [null, null]
  );

  scene.graph = {
    minX,
    maxX,
    ticksX,
    minY,
    maxY
  };

  scene.utils = Utils(viz);

  const yAxis = YAxis(viz, scene);
  const xAxis = XAxis(viz, scene);
  const legend = Legend(viz, scene);

  let plots = [];
  Array.from(viz.data.plot)
    .reverse()
    .forEach((p, i) => {
      let idx = viz.data.plot.length - 1 - i;
      plots.push(Plot(viz, scene, p, viz.meta.colorList[idx % viz.meta.colorList.length]));
    });

  scene.updated = function () {
    yAxis.updated();
    xAxis.updated();
    legend.updated();
    plots.forEach((p) => {
      p.updated();
    });
  };

  return scene;
}
