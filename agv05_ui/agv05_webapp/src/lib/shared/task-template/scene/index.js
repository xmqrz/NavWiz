/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';
import * as d3 from 'd3';
import { Graphviz } from '@hpcc-js/wasm';

import $perf from '$lib/shared/perf';
import EditMode from '../../../../routes/(config-panel)/config/task-templates/add/scene/edit-mode';
import ActiveObject from '../../../../routes/(config-panel)/config/task-templates/add/scene/active-object';
import hq from '$lib/shared/html-escape';

var graphviz;
var onGraphvizReady;
var wasmFolder;
if (window.agvPanelToken) {
  wasmFolder = 'http://localhost/static/app/js';
}

Graphviz.load(wasmFolder).then(function (g) {
  graphviz = g;
  if (onGraphvizReady) {
    onGraphvizReady();
  }
});

var hqq = (s) => hq(hq(s));

function _tpl_node_unicell(id, active, name, sink) {
  var tooltip = `"${name}" Terminal`;
  var fillColor = active ? 'gray' : 'white';
  var rank = sink ? `{rank=sink; ${id}}` : '';
  return `
${id} [shape=rect style="rounded,filled" fillcolor=${fillColor} label=<
  <table border="0" cellborder="0" cellpadding="0" cellspacing="3">
    <tr>
      <td width="150">
<b><font point-size="18">${hq(name)}</font></b>
      </td>
    </tr>
  </table>
> tooltip="${hq(tooltip)}"]
${rank}
`;
}

function _tpl_node_multicell(id, active, action, skill, activeObject, taskTemplateUrl) {
  var isSub = skill.id.indexOf('_ttpk_') === 0;
  var formatLabel = isSub ? _.identity : _.startCase;
  var fillColor = active ? 'limegreen' : 'white';
  var nCols = skill.outcomes.length;

  var displayName;
  var titleFillColor;
  if (skill.id.indexOf('fms:') === 0) {
    displayName = '[DFleet] ' + skill.name;
    titleFillColor = 'lightcoral';
  } else {
    displayName = skill.name;
    titleFillColor = 'khaki';
  }
  var tooltip = `"${displayName}" ${isSub ? 'Task Template' : 'Action'}`;

  var linkTpl = '';
  if (isSub && taskTemplateUrl) {
    linkTpl = taskTemplateUrl.replace('999999999', skill.id.slice(6));
    linkTpl = `
<td width="${18 + displayName.length / 3}" align="right" href="${linkTpl}" target="_blank" title="Open nested task template"><b><font point-size="16">&#x27a6;</font></b></td>
`;
  }

  var paramsTpl = '';
  skill.params.forEach(function (param) {
    var paramName = formatLabel(param.name);
    if (param.version) {
      paramName += ' (v' + param.version + ')';
    }
    var paramValue = action.params[param.key];
    if (paramValue === undefined || paramValue === null) {
      paramValue = '';
    }
    paramsTpl += `<br align="left"/>${hq(paramName)}: ${hq(paramValue)}
`;
  });

  var outcomesTpl = '';
  skill.outcomes.forEach(function (outcome) {
    var displayName = formatLabel(outcome);
    var tooltip = `"${displayName}" Outcome`;
    var fillColor =
      activeObject.get().action_id === id && activeObject.get().outcome_key === outcome
        ? 'slateblue'
        : 'skyblue';
    outcomesTpl += `
<td id="action_${id}_outcome__${hqq(outcome)}" href="javascript:void(0)" bgcolor="${fillColor}" port="${hqq(outcome)}" tooltip="${hq(tooltip)}">
<b>${hq(displayName)}</b>
</td>
`;
  });

  return `
${id} [shape=rect style="rounded,filled" fillcolor=${fillColor} label=<
  <table border="0" cellborder="1" cellpadding="5" cellspacing="5">
    <tr>
      <td colspan="${nCols}" width="300" bgcolor="${titleFillColor}">
        <table border="0" cellborder="0" cellpadding="0" cellspacing="0">
          <tr>
<td width="288"><b><font point-size="16">${hq(displayName)}</font></b></td>${linkTpl}
          </tr>
        </table>
      </td>
    </tr>
    <tr>
      <td colspan="${nCols}" bgcolor="wheat">
<b>Parameters:</b>
${paramsTpl}
      <br align="left"/></td>
    </tr>
    <tr>
${outcomesTpl}
    </tr>
  </table>
> tooltip="${hq(tooltip)}"]
`;
}

function _tpl_edge(from, port, to, active) {
  var color = active ? 'red' : 'black';
  var tooltip = port ? `"${port}" Outcome` : 'Start';
  port = port ? `:"${hq(port)}"` : '';
  return `${from}${port}->${to}[color=${color} tooltip="${hq(tooltip)}"]
`;
}

function _tpl_graph(
  actions,
  skillDescriptions,
  taskTemplateMetas,
  outcomes,
  activeObject,
  preserve,
  taskTemplateUrl
) {
  var nodesTpl = '';
  var edgesTpl = '';

  // Start terminals
  nodesTpl += _tpl_node_unicell(0, activeObject.isStartTerminal(), 'Start');

  // Nodes and Edges
  for (let idx = 0; idx < actions.length; idx++) {
    let action = actions[idx];
    if (idx === 0) {
      if (action) {
        edgesTpl += _tpl_edge(0, null, action, activeObject.isStartTerminal());
      }
      continue;
    }

    if (!action || !action.skillId) {
      continue;
    }
    var skill;
    if (action.skillId.indexOf('_ttpk_') === 0) {
      skill = taskTemplateMetas[action.skillId];
    } else {
      skill = skillDescriptions[action.skillId];
    }
    if (preserve) {
      skill = skill || {};
      skill.id = skill.id || action.skillId;
      skill.name = skill.name || action.skillId;
      skill.params = [];
      for (var k in action.params) {
        if (action.skillId.indexOf('_ttpk_') !== 0) {
          var m = k.match(/^(.*):([1-9]\d*)$/);
          if (m) {
            skill.params.push({
              name: m[1],
              version: parseInt(m[2]),
              key: k
            });
            continue;
          }
        }
        skill.params.push({
          name: k,
          key: k
        });
      }
      skill.outcomes = Object.keys(action.outcomes);
    }
    if (!skill || !skill.name || !skill.outcomes) {
      continue;
    }

    nodesTpl += _tpl_node_multicell(
      idx,
      activeObject.get().action_id === idx,
      action,
      skill,
      activeObject,
      taskTemplateUrl
    );

    for (var key in action.outcomes) {
      var target = action.outcomes[key];
      if (!target) {
        continue;
      }
      edgesTpl += _tpl_edge(
        idx,
        key,
        target,
        activeObject.get().action_id === idx && activeObject.get().outcome_key === key
      );
    }
  }

  // End terminal
  for (let idx = 0; idx < outcomes.length; idx++) {
    nodesTpl += _tpl_node_unicell(~idx, false, outcomes[idx], true);
  }

  return `
digraph "TaskTemplate" {
  rankdir=UD
  bgcolor=transparent
${nodesTpl}
${edgesTpl}
}
`;
}

export default function (viz) {
  /**
   * Layer: Scene
   */
  var scene = viz.append('g');
  scene.attr('class', 'scene');

  scene.overlay = scene.append('g');
  scene.overlay.attr('class', 'overlay');

  scene.activeObject = ActiveObject(viz, scene);

  scene.zoomed = function () {
    scene.attr('transform', `translate(${viz.meta.translate})scale(${viz.meta.scale})`);
  };

  scene.resized = function () {};

  scene.modelsUpdated = function () {
    // If graphviz not ready, register callback and return.
    if (!graphviz) {
      onGraphvizReady = scene.modelsUpdated;
      return;
    }

    // Clear existing scene
    scene.select('g#graph0').remove();

    // Retrieve data from models
    var actions = viz.models.rawActions();
    var outcomes = viz.models.rawOutcomes();
    var skillDescriptions = viz.models.skillDescriptions();
    var taskTemplateMetas = viz.models.taskTemplateMetas();
    var activeObject = scene.activeObject;
    var graph;

    var dot_g = _tpl_graph(
      actions,
      skillDescriptions,
      taskTemplateMetas,
      outcomes,
      activeObject,
      viz.options.preserve,
      viz.options.taskTemplateUrl
    );
    $perf.begin('task-template.render-viz');
    var g;
    try {
      g = graphviz.layout(dot_g, 'svg', 'dot');
    } catch (e) {
      window.alert('Out-of-memory');
      return;
    }
    $perf.end('task-template.render-viz');

    graph = d3
      .select(new DOMParser().parseFromString(g, 'image/svg+xml').documentElement)
      .select('g.graph');

    scene.node().prepend(graph.node());

    scene.graphUpdated(graph);
  };

  scene.graphUpdated = function (graph) {
    scene.overlay.attr('transform', graph.attr('transform'));
  };

  scene.mode = EditMode.POINTER;
  scene.setEditMode = function (m) {
    scene.mode = m;
  };

  return scene;
}
