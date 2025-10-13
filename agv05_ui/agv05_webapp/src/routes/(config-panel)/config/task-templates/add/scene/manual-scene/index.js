/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as d3 from 'd3';
import * as _ from 'lodash-es';
import terminalsF from './terminals';
import nodesF from './nodes';
import edgesF from './edges';
import constructF from './construct';

var TERMINAL_PAD = 100;

export default function (viz, scene) {
  var curTranslateX;
  var curTranslateY;

  var manualScene = d3.select(document.createElementNS('http://www.w3.org/2000/svg', 'g'));
  manualScene.attr('class', 'task-template-manual-scene');
  manualScene.overlay = scene.overlay;

  var terminals = terminalsF(manualScene);
  var nodes = nodesF(manualScene);
  var edges = edgesF(manualScene, manualScene);

  var construct;
  if (viz.options.editable) {
    construct = constructF(viz, manualScene);
  }

  function modelsUpdated(actions, outcomes, activeObject) {
    let dragOb;
    if (viz.options.editable) {
      dragOb = construct.getDragOb();
    }

    let outcomesXList = {};

    let minY = Number.POSITIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let minX = 0;

    let nodesData = [];
    let edgesData = [];

    actions.slice(1).forEach(function (action, idx) {
      if (
        !action.renderMeta ||
        !action.renderMeta.outcomes ||
        action.renderMeta.outcomes.length <= 0
      ) {
        return;
      }

      let actionMinY = action.pos[1] - action.renderMeta.height / 2;
      let actionMaxY = action.pos[1] + action.renderMeta.height / 2;
      let actionMinX = action.pos[0] - action.renderMeta.width / 2;
      if (minY > actionMinY) {
        minY = actionMinY;
      }
      if (maxY < actionMaxY) {
        maxY = actionMaxY;
      }
      if (minX > actionMinX) {
        minX = actionMinX;
      }

      // TODO: can be merge with bottom loop.
      action.renderMeta.outcomes.forEach(function (outcome, i) {
        let outIdx = action.outcomes[outcome];
        if (outIdx < 0) {
          if (!outcomesXList[~outIdx]) {
            outcomesXList[~outIdx] = [];
          }
          let outcomeX = action.pos[0] + action.renderMeta.outcomesRelativePos[i][0];
          outcomesXList[~outIdx].push(outcomeX);
        }
      });

      let title = idx + 1;
      let active = activeObject.get().action_id === title;
      let activeOutcome = active && activeObject.get().outcome_key;
      let id = `node${idx + 2}`;

      let actionOutcomes = action.renderMeta.outcomes.map(function (outcome, idx) {
        let pos = action.renderMeta.outcomesRelativePos[idx];
        return {
          id: `a_action_${title}_outcome__${outcome}`,
          name: action.renderMeta.outcomesText[idx],
          width: action.renderMeta.outcomesWidth[idx],
          height: action.renderMeta.outcomeHeight,
          active: active && activeOutcome === outcome,
          x: action.pos[0] + pos[0],
          y: action.pos[1] + pos[1],
          target: action.outcomes[outcome],
          val: outcome
        };
      });

      nodesData.push({
        name: action.renderMeta.displayName,
        active: active,
        activeOutcome: activeOutcome,
        drag: dragOb && dragOb.action_id === title,
        float: false,
        cursor: active ? 'grab' : 'default',
        id: id,
        title: title,
        width: action.renderMeta.width,
        height: action.renderMeta.height,
        paramWidth: action.renderMeta.paramWidth,
        paramsText: action.renderMeta.paramsText,
        outcomes: actionOutcomes,
        x: action.pos[0],
        y: action.pos[1],
        url: action.renderMeta.url,
        color: action.renderMeta.color
      });
    });

    minY = minY === Number.POSITIVE_INFINITY ? -TERMINAL_PAD : minY;
    maxY = maxY === Number.NEGATIVE_INFINITY ? TERMINAL_PAD : maxY;

    let terminalsData = [
      {
        active: activeObject.get().action_id === 0,
        id: 'node1',
        name: 'Start',
        x: actions[0] ? actions[actions[0]].pos[0] : 0,
        y: minY - TERMINAL_PAD,
        title: 0,
        height: 36,
        width: 172
      }
    ];

    let outcomePadding = 6;
    outcomes.forEach(function (name, idx) {
      let title = ~idx;
      let xList = outcomesXList[idx];
      let width = textWidth(name, 18, 'bold') + 30;
      width = width < 172 ? 172 : width;
      terminalsData.push({
        active: activeObject.get().action_id === title,
        id: `node${actions.length + idx + 1}`,
        title: title,
        name: name,
        x: xList ? xList.reduce((a, b) => a + b, 0) / xList.length : undefined,
        y: maxY + TERMINAL_PAD,
        height: 36,
        width: width
      });
    });
    let outcomesMaxX = resolveHorizontalOverlap(
      terminalsData.slice(1).filter((d) => d.x !== undefined),
      outcomePadding
    );
    terminalsData
      .slice(1)
      .filter((d) => d.x === undefined)
      .forEach((d) => {
        d.x = outcomesMaxX + outcomePadding + d.width / 2;
        outcomesMaxX = d.x + d.width / 2 + outcomePadding;
      });
    terminalsData.forEach((d) => {
      if (minX > d.x - d.width / 2) {
        minX = d.x - d.width / 2;
      }
    });

    if (actions[0] && (!dragOb || dragOb.action_id !== actions[0])) {
      let startTerminal = terminalsData[0];
      let action = actions[actions[0]];
      edgesData.push({
        srcPos: [startTerminal.x, startTerminal.y + startTerminal.height / 2],
        dstPos: action.pos,
        dstDim: [action.renderMeta.width, action.renderMeta.height],
        active: activeObject.get().action_id === 0,
        loop: false,
        title: `0->${actions[0]}`
      });
    }

    nodesData.forEach(function (node) {
      if (dragOb && dragOb.action_id === node.title) {
        return;
      }
      node.outcomes.forEach(function (outcome) {
        if (!outcome.target) {
          return;
        }
        if (dragOb && dragOb.action_id === outcome.target) {
          return;
        }

        if (outcome.target > 0) {
          let action = actions[outcome.target];
          edgesData.push({
            srcPos: [outcome.x, outcome.y + outcome.height / 2],
            dstPos: action.pos,
            dstDim: [action.renderMeta.width, action.renderMeta.height],
            active: outcome.active,
            loop: outcome.target === node.title,
            title: `${node.title}:${outcome.val}->${outcome.target}`
          });
        } else {
          let terminal = terminalsData[Math.abs(outcome.target)];

          edgesData.push({
            srcPos: [outcome.x, outcome.y + outcome.height / 2],
            dstPos: [terminal.x, terminal.y],
            dstDim: [terminal.width, terminal.height],
            active: outcome.active,
            loop: false,
            title: `${node.title}:${outcome.val}->${outcome.target}`
          });
        }
      });
    });

    terminals.modelsUpdated(terminalsData);
    nodes.modelsUpdated(nodesData);
    edges.modelsUpdated(edgesData, nodesData);

    let newTranslateX = -minX + 10;
    let newTranslateY = terminalsData[0].height / 2 - terminalsData[0].y + 10;

    // Translate viz zoom based on manualScene size.
    if (curTranslateX) {
      let translateDX = newTranslateX - curTranslateX;
      let translateDY = newTranslateY - curTranslateY;
      translateDX *= viz.meta.scale;
      translateDY *= viz.meta.scale;
      viz.zoom.zoomTranslate(
        viz.meta.translate[0] - translateDX,
        viz.meta.translate[1] - translateDY
      );
    }
    manualScene.attr('transform', `translate(${newTranslateX},${newTranslateY})`);

    curTranslateX = newTranslateX;
    curTranslateY = newTranslateY;

    viz.meta.manualScene.minY = minY;
    viz.meta.manualScene.maxY = maxY;
    viz.meta.manualScene.minX = minX;
  }

  function updateRenderMetadata(action) {
    let skillDescriptions = viz.models.skillDescriptions();
    let taskTemplateMetas = viz.models.taskTemplateMetas();
    let preserve = viz.options.preserve;
    let taskTemplateUrl = viz.options.taskTemplateUrl;

    if (!action || !action.skillId) {
      return;
    }
    let skill;
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
            return;
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
      return;
    }

    let isSub = skill.id.indexOf('_ttpk_') === 0;
    let formatLabel = isSub ? _.identity : _.startCase;

    let paramWidth = textWidth('Parameters:', 14, 'bold');
    let paramsText = [];
    for (let p of skill.params) {
      var paramName = formatLabel(p.name);
      if (p.version) {
        paramName += ' (v' + p.version + ')';
      }
      var paramValue = action.params[p.key];
      if (paramValue === undefined || paramValue === null) {
        paramValue = '';
      }
      let text = `${paramName}: ${paramValue}`;

      paramsText.push(text);
      let textLen = textWidth(text);
      if (textLen > paramWidth) {
        paramWidth = textLen;
      }
    }

    let totalOutcomesWidth = 0;
    let outcomesWidth = [];
    let outcomesText = [];
    for (let i in skill.outcomes) {
      let outcome = skill.outcomes[i];
      let text = formatLabel(outcome);
      let textLen = textWidth(text, 14, 'bold');
      totalOutcomesWidth += textLen;
      outcomesWidth.push(textLen);
      outcomesText.push(text);
    }

    let displayName = skill.name;
    let titleWidth = textWidth(displayName, 16, 'bold');

    let width = 326 + 5 * skill.outcomes.length;
    if (titleWidth + 36 > width) {
      width = titleWidth + 36;
    }
    if (paramWidth + 36 > width) {
      width = paramWidth + 36;
    }

    let padBetweenOutcome = 5 * (skill.outcomes.length - 1);
    let textPad = 4;
    let totalTextPad = textPad * skill.outcomes.length;

    if (totalOutcomesWidth + 36 + padBetweenOutcome + totalTextPad > width) {
      width = totalOutcomesWidth + 36 + padBetweenOutcome + totalTextPad;
    }

    let height = 118 + 16 * paramsText.length;
    let paramHeight = height - 10 - 26 - 31 - 28;

    let outcomeHeight = 28;
    let renderOWidth = width - 26 - padBetweenOutcome - totalTextPad;
    let y = height / 2 - 13 - outcomeHeight / 2;
    let leftMost = -width / 2 + 13;
    let x = leftMost - 5;
    let outcomesRelativePos = [];
    let curWidth = 0;
    outcomesWidth = outcomesWidth.map(function (oWidth) {
      x += curWidth / 2 + 5;
      curWidth = (oWidth / totalOutcomesWidth) * renderOWidth + textPad;
      x += curWidth / 2;
      outcomesRelativePos.push([x, y]);
      return curWidth;
    });

    let url = '';
    if (isSub && taskTemplateUrl) {
      url = taskTemplateUrl.replace('999999999', skill.id.slice(6));
    }

    Object.assign(action.renderMeta, {
      width: width,
      height: height,
      displayName: displayName,
      titleWidth: titleWidth,
      titleHeight: 31,
      paramWidth: paramWidth,
      paramHeight: paramHeight,
      paramsText: paramsText,
      outcomeWidth: totalOutcomesWidth,
      outcomeHeight: 28,
      outcomesRelativePos: outcomesRelativePos,
      outcomesWidth: outcomesWidth,
      outcomesText: outcomesText,
      outcomes: skill.outcomes,
      url: url,
      color: 'khaki'
    });
  }

  // Utils
  var textWidthUtil = viz.append('text');
  textWidthUtil.attr('font-family', 'Times,serif');

  function textWidth(text, fontSize = 14, fontWeight = 'normal') {
    textWidthUtil
      .attr('x', -100)
      .attr('y', -100)
      .attr('font-weight', fontWeight)
      .attr('font-size', fontSize)
      .text(text);
    return textWidthUtil.node().getComputedTextLength();
  }
  manualScene.textWidth = textWidth;

  function resolveHorizontalOverlap(data, padding) {
    if (data.length <= 0) {
      return 0;
    }
    let delta = 2;
    let maxX = 0;
    let overlap = true;
    let attempt = 30000;
    while (overlap && attempt > 0) {
      attempt -= 1;
      overlap = false;
      data = data.sort((a, b) => a.x - b.x);
      let max = data[data.length - 1];
      maxX = max.x + max.width / 2 + padding;
      for (let i = 0; i < data.length - 1; i++) {
        let cur = data[i];
        let next = data[i + 1];
        if (horizontalOverlap(cur, next, padding)) {
          overlap = true;
          cur.x -= delta;
          next.x += delta;
        }
      }
    }

    return maxX;
  }

  function horizontalOverlap(a, b, padding) {
    let minA = a.x - a.width / 2 - padding;
    let maxA = a.x + a.width / 2 + padding;
    let minB = b.x - b.width / 2 - padding;
    let maxB = b.x + b.width / 2 + padding;
    return maxA > minB && maxB > minA;
  }

  return {
    graph: () => manualScene,
    modelsUpdated: modelsUpdated,
    updateRenderMetadata: updateRenderMetadata
  };
}
