/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as d3 from 'd3';
import nodesF from './nodes';

export default function (viz, manualScene) {
  var dragOb;

  var construct = manualScene.overlay.append('g').attr('class', 'manual-construct');
  var nodes = nodesF(construct);

  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);

  function trackstarted(event) {
    /* jshint validthis: true */
    dragOb = null;
    let ob = findDraggableNode(event.trackTarget);
    if (!ob) {
      return;
    }

    let actions = viz.models.rawActions();
    let action = actions[ob.action_id];
    dragOb = {
      action_id: ob.action_id,
      elem: ob.elem,
      x: action.pos[0],
      y: action.pos[1],
      origX: action.pos[0],
      origY: action.pos[1]
    };

    viz.style('cursor', 'grabbing');
    drawDraggables();
    if (window.TouchEvent && event instanceof TouchEvent) {
      // TouchEvent required the target to stay exist
    } else {
      viz.modelsUpdated();
    }
    event.stopPropagation();
  }

  function tracked() {
    /* jshint validthis: true */
    if (!dragOb) {
      return;
    }
    let x = this.p1[0] - this.p0[0];
    let y = this.p1[1] - this.p0[1];

    dragOb.x = x + dragOb.origX;
    dragOb.y = y + dragOb.origY;

    drawDraggables();
  }

  function trackended() {
    /* jshint validthis: true */
    if (!dragOb) {
      return;
    }
    viz.style('cursor', 'default');
    viz.models.push('updateActionPos');

    let actions = viz.models.rawActions();
    let action = actions[dragOb.action_id];
    action.pos = [dragOb.x, dragOb.y];

    dragOb = null;
    drawDraggables();
    viz.models.updateAction(action);
  }

  function drawDraggables() {
    if (!dragOb) {
      nodes.modelsUpdated([]);
      return;
    }

    var actions = viz.models.rawActions();

    let action = actions[dragOb.action_id];

    let actionOutcomes = action.renderMeta.outcomes.map(function (outcome, idx) {
      let pos = action.renderMeta.outcomesRelativePos[idx];
      return {
        id: `a_action_${dragOb.action_id}_outcome__${outcome}__draggable`,
        name: action.renderMeta.outcomesText[idx],
        width: action.renderMeta.outcomesWidth[idx],
        height: action.renderMeta.outcomeHeight,
        active: false,
        x: dragOb.x + pos[0],
        y: dragOb.y + pos[1],
        target: action.outcomes[outcome],
        val: outcome
      };
    });

    let nodesData = [
      {
        name: action.renderMeta.displayName,
        active: true,
        activeOutcome: null,
        drag: false,
        float: true,
        cursor: 'grabbing',
        id: 'draggable-node',
        title: dragOb.action_id,
        width: action.renderMeta.width,
        height: action.renderMeta.height,
        paramWidth: action.renderMeta.paramWidth,
        paramsText: action.renderMeta.paramsText,
        outcomes: actionOutcomes,
        x: dragOb.x,
        y: dragOb.y,
        url: action.renderMeta.url,
        color: action.renderMeta.color
      }
    ];

    nodes.modelsUpdated(nodesData);
  }

  manualScene.call(track);

  // utils
  function findDraggableNode(target) {
    let activeObject = viz.scene.activeObject.get();

    if (!activeObject.action_id || activeObject.outcome_key) {
      return;
    }

    let $target = $(target);
    if ($target.closest('a').length > 0) {
      return;
    }

    let node = $target.closest('.node');
    if (node.length <= 0) {
      return;
    }

    let outcome = $target.closest('.outcome');
    if (outcome.length > 0) {
      return;
    }

    let title = node.find('title');
    if (title.length <= 0) {
      return;
    }

    let action_id = parseInt(title.text());

    if (action_id <= 0 || action_id !== activeObject.action_id) {
      return;
    }

    return {
      action_id: action_id,
      elem: node[0]
    };
  }

  return {
    getDragOb: () => dragOb
  };
}
