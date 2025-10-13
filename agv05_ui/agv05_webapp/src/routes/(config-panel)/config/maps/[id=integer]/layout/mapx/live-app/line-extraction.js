/*
 * Copyright (c) 2025, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

// ChatGPT conversion of ros line line extraction package.
// TODO: cleanup and remove unused code.

import * as ML from 'ml-matrix';
import * as math from 'mathjs';

const defaultParams = {
  min_range: 0.05,
  max_range: 8.0,
  outlier_dist: 0.1,
  min_split_dist: 0.05,
  max_line_gap: 0.5,
  min_line_points: 4
};

// Helper: Polar to Cartesian
function polarToCartesian(ranges, cosBearings, sinBearings) {
  let c = new Array(ranges.length);
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    c[i] = {
      index: i,
      x: r * cosBearings[i],
      y: r * sinBearings[i],
      range: r
    };
  }
  return c;
}

// Helper: Point-to-Line Distance
function pointToLineDistance(p, a, b) {
  const num = Math.abs((b.y - a.y) * p.x - (b.x - a.x) * p.y + b.x * a.y - b.y * a.x);
  const den = Math.hypot(b.y - a.y, b.x - a.x);
  return den === 0 ? 0 : num / den;
}

// Outlier Filtering
function filterOutliers(points, params) {
  const out = [];
  for (let i = 1; i < points.length - 1; i++) {
    const p = points[i];
    const prev = points[i - 1];
    const next = points[i + 1];
    if (
      Math.abs(p.range - prev.range) > params.outlier_dist &&
      Math.abs(p.range - next.range) > params.outlier_dist
    ) {
      const d = pointToLineDistance(p, prev, next);
      if (d > params.min_split_dist) continue;
    }
    out.push(p);
  }
  return out;
}

// Least Squares Line Fit using ml-matrix SVD
function fitLine(points) {
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const mx = math.mean(xs);
  const my = math.mean(ys);

  const A = points.map((p) => [p.x - mx, p.y - my]);
  const svd = new ML.SVD(A, {
    computeLeftSingularVectors: false,
    computeRightSingularVectors: true
  });

  const dir = svd.rightSingularVectors.getColumn(0);
  const rad = Math.atan2(dir[1], dir[0]);
  const radius = mx * Math.cos(rad) + my * Math.sin(rad);

  // Approximate 2x2 covariance
  const cov = [
    [0.001, 0],
    [0, 0.001]
  ];

  return { rad, radius, covariance: cov };
}

// Split and Merge
function split(points, lines, params) {
  if (points.length < params.min_line_points) return;

  const first = points[0];
  const last = points[points.length - 1];
  let maxDist = 0,
    iSplit = -1;

  for (let i = 1; i < points.length - 1; i++) {
    const d = pointToLineDistance(points[i], first, last);
    if (d > maxDist) {
      maxDist = d;
      iSplit = i;
    }
  }

  let maxGap = 0;
  for (let i = 0; i < points.length - 1; i++) {
    const gap = math.hypot(points[i].x - points[i + 1].x, points[i].y - points[i + 1].y);
    if (gap > maxGap) maxGap = gap;
  }

  if (maxDist < params.min_split_dist && maxGap < params.max_line_gap) {
    const model = fitLine(points);
    lines.push({
      ...model,
      start: first,
      end: last,
      indices: points.map((p) => p.index),
      points
    });
  } else {
    split(points.slice(0, iSplit), lines, params);
    split(points.slice(iSplit), lines, params);
  }
}

function chiSquared(dL, P1, P2) {
  const P = math.add(P1, P2);
  const P_inv = math.inv(P);
  const dL_row = math.matrix([dL]); // 1×2
  const dL_col = math.transpose(dL_row); // 2×1
  const tmp = math.multiply(dL_row, P_inv); // 1×2 × 2×2 = 1×2
  const result = math.multiply(tmp, dL_col); // 1×2 × 2×1 = 1×1
  return result.get([0, 0]); // scalar
}

function mergeLines(lines) {
  const merged = [];
  let i = 0;
  while (i < lines.length) {
    let current = lines[i];
    let j = i + 1;

    while (j < lines.length) {
      const next = lines[j];
      const L1 = [current.radius, current.rad];
      const L2 = [next.radius, next.rad];
      const dL = math.subtract(L1, L2);
      const chi2 = chiSquared(dL, current.covariance, next.covariance);

      if (chi2 < 3) {
        const P1_inv = math.inv(current.covariance);
        const P2_inv = math.inv(next.covariance);
        const Pm = math.inv(math.add(P1_inv, P2_inv));
        const Lm = math.multiply(
          Pm,
          math.add(math.multiply(P1_inv, L1), math.multiply(P2_inv, L2))
        );
        current = {
          radius: Lm[0],
          rad: Lm[1],
          covariance: Pm,
          start: current.start,
          end: next.end,
          indices: [...current.indices, ...next.indices]
        };
        j++;
      } else {
        break;
      }
    }

    merged.push(current);
    i = j;
  }

  return merged;
}

function extractLines(ranges, cosBearings, sinBearings, params) {
  const raw = polarToCartesian(ranges, cosBearings, sinBearings);
  const filtered = filterOutliers(
    raw.filter((p) => p.range > params.min_range && p.range < params.max_range),
    params
  );
  let lines = [];
  split(filtered, lines, params);
  lines = mergeLines(lines);
  return lines;
}

function genAngleArray(radMin, radMax, radInc) {
  let angles = [];
  for (let rad = radMin; rad <= radMax; rad += radInc) {
    angles.push(rad);
  }
  return angles;
}

export default function (ranges, radMin, radMax, radInc, params = {}) {
  let angles = genAngleArray(radMin, radMax, radInc);
  let cosBearings = angles.map(Math.cos);
  let sinBearings = angles.map(Math.sin);
  params = Object.assign({}, defaultParams, params);
  return extractLines(ranges, cosBearings, sinBearings, params);
}
