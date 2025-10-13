/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';

export default function (viz, scene, muted = false) {
  /* Paths */
  var paths;
  if (muted) {
    paths = scene.append('g').attr('class', 'paths-muted');
  } else {
    var branches = scene.append('g').attr('class', 'path-branches');
    paths = scene.append('g').attr('class', 'paths');
    var alignments = scene.append('g').attr('class', 'path-alignments');
  }
  var Utils = scene.Utils;

  function modelsUpdated() {
    var visibleLayers = viz.models.rawVisibleLayers();
    var data = viz.models.rawPaths();
    var dataJ = viz.models.rawJunctions();
    var activeObjects = scene.activeObject.getAll();
    var hoverObjects = scene.hoverObject.getAll();
    var discardObjects = scene.discardObject.getAll();
    var aoAligned = scene.activeObject.getAligned();
    var hoAligned = scene.hoverObject.getAligned();
    var aoBranched = scene.activeObject.getBranched();
    var hoBranched = scene.hoverObject.getBranched();
    var p = paths
      .selectAll('path.path')
      .data(data.filter((p) => visibleLayers[p.layer] !== muted))
      .join('path');
    p.attrs({
      d: function (d) {
        if (d.j1 >= 0 && d.j1 < dataJ.length && d.j2 >= 0 && d.j2 < dataJ.length) {
          return Utils.generateConnectorPath(d, dataJ[d.j1], dataJ[d.j2]);
        } else {
          console.log("Path's junction index out of bounds.");
          return '';
        }
      },
      class: (d) => `path layer-${d.layer}`
    })
      .classed('path-dynamic', (d) => d.dynamic)
      .classed('muted', muted)
      .classed('active', function (d) {
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      });

    // overlay rail pattern if the path is tracked
    var r = paths
      .selectAll('path.path-rail')
      .data(
        _.filter(
          data.filter((p) => visibleLayers[p.layer] !== muted),
          'tracked'
        )
      )
      .join('path');
    r.attrs({
      class: (d) => `path-rail layer-${d.layer}`,
      d: function (d) {
        if (d.j1 >= 0 && d.j1 < dataJ.length && d.j2 >= 0 && d.j2 < dataJ.length) {
          return Utils.generateConnectorPath(d, dataJ[d.j1], dataJ[d.j2], true);
        } else {
          console.log("Path's junction index out of bounds.");
          return '';
        }
      }
    })
      .classed('muted', muted)
      .classed('active', function (d) {
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      });

    if (muted) {
      return;
    }

    // Path alignments
    var alignedPaths = {};
    if (aoAligned.ob) {
      var aoIdx = data.indexOf(scene.activeObject.get());
      if (aoIdx >= 0) {
        alignedPaths[aoIdx] |= aoAligned.ob;
      }
    }
    if (hoAligned.ob) {
      var hoIdx = data.indexOf(scene.hoverObject.get());
      if (hoIdx >= 0) {
        alignedPaths[hoIdx] |= hoAligned.ob;
      }
    }
    _.mergeWith(alignedPaths, aoAligned.paths, hoAligned.paths, (a, b) => a | b);
    alignedPaths = _.reduce(
      alignedPaths,
      function (result, value, key) {
        let d = data[key];
        if (visibleLayers[d.layer]) {
          result.push([data[key], value]);
        }
        return result;
      },
      []
    );
    var a = alignments.selectAll('path.path-aligned').data(alignedPaths).join('path');
    a.attr('class', 'path-aligned');
    a.attrs({
      d: function (d) {
        d = d[0];
        if (d.j1 >= 0 && d.j1 < dataJ.length && d.j2 >= 0 && d.j2 < dataJ.length) {
          return Utils.generateConnectorPath(d, dataJ[d.j1], dataJ[d.j2], true);
        } else {
          console.log("Path's junction index out of bounds.");
          return '';
        }
      }
    })
      .classed('active', function (d) {
        d = d[0];
        return (
          discardObjects.indexOf(d) < 0 &&
          (activeObjects.indexOf(d) >= 0 || hoverObjects.indexOf(d) >= 0)
        );
      })
      .styles({
        'stroke-dasharray': function (d) {
          var dynamic = d[0].dynamic;
          d = d[1];
          var len = this.getTotalLength();
          var len3 = Math.min(len / 3.0, 0.6);
          var l1 = d & 1 ? len3 : 0;
          var l2 = d & 2 ? len3 : 0;

          if (!dynamic) {
            return `${l1} ${len - l1 - l2} ${l2}`;
          }

          var dash = 0.1; // must corresponds to dynamic path's stroke-dasharray

          var n1 = Math.floor(l1 / dash);
          var r1 = l1 % dash;
          var s = '0.1 '.repeat(n1); // repeat dash and gap for l1
          if (n1 & 1) {
            // odd n1, reduce l1 and thus increase the middle gap by r1
            l1 -= r1;
          } else {
            // even n1, add a partial dash
            s += r1 + ' ';
          }

          var l = len - l2;
          var n = Math.ceil(l / dash) - 1;
          var r = dash - (l % dash);
          if (n & 1) {
            // odd n, reduce l2 and thus increase the middle gap by r
            l2 -= r;
            s += `${len - l1 - l2}`;
          } else {
            // even n, add middle gap and a partial dash
            s += `${len - l1 - l2} ${r}`;
            l2 -= r;
          }

          var n2 = Math.ceil(l2 / dash);
          s += ' 0.1'.repeat(n2); // repeat dash and gap for l2
          return s;
        }
      });

    // Path branched
    var branchedPaths = {};
    _.mergeWith(branchedPaths, aoBranched, hoBranched, (a, b) => a | b);
    branchedPaths = _.reduce(
      branchedPaths,
      function (result, value, key) {
        let d = data[key];
        if (visibleLayers[d.layer]) {
          result.push([data[key], value]);
        }
        return result;
      },
      []
    );
    var b = branches.selectAll('path.path-branched').data(branchedPaths).join('path');
    b.attr('class', 'path-branched');
    b.attrs({
      d: function (d) {
        d = d[0];
        if (d.j1 >= 0 && d.j1 < dataJ.length && d.j2 >= 0 && d.j2 < dataJ.length) {
          return Utils.generateConnectorPath(d, dataJ[d.j1], dataJ[d.j2], true);
        } else {
          console.log("Path's junction index out of bounds.");
          return '';
        }
      }
    }).styles({
      'stroke-dasharray': function (d) {
        d = d[1];
        var len = this.getTotalLength();
        var len3 = Math.min(len / 3.0, 0.8);
        var l1 = d & 0b01 ? len3 : 0;
        var l2 = d & 0b10 ? len3 : 0;

        return `${l1} ${len - l1 - l2} ${l2}`;
      }
    });
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
