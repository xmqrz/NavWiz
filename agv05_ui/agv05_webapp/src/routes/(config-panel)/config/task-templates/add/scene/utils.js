/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

export default function (viz, _scene) {
  function findOutcome(target) {
    let $target = $(target);
    let node = $target.closest('.node');
    if (node.length <= 0) {
      return;
    }

    if (node.attr('id') === 'node1') {
      return {
        action_id: 0,
        outcome_key: 'Start',
        elem: node[0]
      };
    }

    let outcome = $target.closest('g');
    if (outcome.length <= 0) {
      return;
    }

    if (!(outcome.attr('id') || '').includes('outcome')) {
      return;
    }

    try {
      let m = outcome.attr('id').match(/action_(\d+)_outcome__(.+)/);
      let action_id = parseInt(m[1]);
      let outcome_key = m[2];
      return {
        action_id: action_id,
        outcome_key: outcome_key,
        elem: outcome[0]
      };
    } catch (error) {
      /* empty */
    }
  }

  function findNode(target) {
    // find node except the start node as it cannot be point to.
    let $target = $(target);
    let node = $target.closest('.node');

    if (node.length <= 0 || node.attr('id') === 'node1') {
      return;
    }

    let outcome = $target.closest('g');
    if (outcome.length > 0 && (outcome.attr('id') || '').includes('outcome')) {
      return;
    }

    try {
      let actions = viz.models.rawActions();
      let m = node.attr('id').match(/node(\d+)/);
      let action_id = parseInt(m[1]) - 1;
      if (action_id >= actions.length) {
        action_id = actions.length - action_id - 1;
      }
      return {
        action_id: action_id,
        elem: node[0]
      };
    } catch (error) {
      /* empty */
    }
  }

  function generateConnectorPath(srcRect, dstRect) {
    let cpDist = 100;
    let j1 = {
      x: srcRect.x + srcRect.width / 2,
      y: srcRect.y + srcRect.height
    };
    let j2 = {
      x: dstRect.x + dstRect.width / 2,
      y: dstRect.y
    };
    let cp1 = {
      x: j1.x,
      y: j1.y + cpDist
    };
    let cp2 = {
      x: j2.x,
      y: j2.y - cpDist
    };

    if (j1.y > j2.y) {
      if (j1.x > j2.x) {
        // left side
        j1 = {
          x: srcRect.x + 10,
          y: srcRect.y + srcRect.height
        };
        cp1 = {
          x: j1.x - cpDist,
          y: j1.y + cpDist
        };
      } else {
        j1 = {
          x: srcRect.x + srcRect.width - 10,
          y: srcRect.y + srcRect.height
        };
        cp1 = {
          x: j1.x + cpDist,
          y: j1.y + cpDist
        };
      }
    }

    return `M ${j1.x},${j1.y} C ${cp1.x},${cp1.y},${cp2.x},${cp2.y},${j2.x},${j2.y}`;
  }

  function generateArrowHead(dstRect) {
    let j2 = {
      x: dstRect.x + dstRect.width / 2,
      y: dstRect.y
    };
    return `${3.5 + j2.x},${-10.0 + j2.y} ${j2.x},${j2.y} ${-3.5 + j2.x},${-10.0 + j2.y} ${
      3.5 + j2.x
    },${-10.0 + j2.y}`;
  }

  function appendConnectorArrow(outer, srcRect, dstRect) {
    outer
      .append('path')
      .attr('d', generateConnectorPath(srcRect, dstRect))
      .attr('stroke', 'red')
      .attr('stroke-width', 3)
      .attr('fill', 'none');
    outer
      .append('polygon')
      .attr('fill', 'red')
      .attr('stroke', 'red')
      .attr('stroke-width', 3)
      .attr('points', generateArrowHead(dstRect));
  }

  var textUtil = viz
    .append('text')
    .attr('class', 'text-util')
    .attr('x', -100)
    .attr('y', -100)
    .attr('font-family', 'Times,serif');

  function textWidth(text, fontSize = 14, fontWeight = 'normal') {
    textUtil.attr('font-weight', fontWeight).attr('font-size', fontSize).text(text);
    return textUtil.node().getComputedTextLength();
  }

  return {
    findOutcome: findOutcome,
    findNode: findNode,
    appendConnectorArrow: appendConnectorArrow,
    textWidth: textWidth
  };
}
