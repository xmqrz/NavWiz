/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

function bindHitTest(sceneElem) {
  $(sceneElem).on('click', onClick);
}

function unbindHitTest(sceneElem) {
  $(sceneElem).off('click');
}

function onClick(evt) {
  var $target = $(evt.target);
  var node = $target.closest('.node');
  if (node.length === 1) {
    return onClickNode(evt, node);
  }

  var edge = $target.closest('.edge');
  if (edge.length === 1) {
    return onClickEdge(evt, edge);
  }
}

function onClickNode(evt, node) {
  var $target = $(evt.target);
  var $title = $(node).find('title');
  if ($title.length !== 1) {
    return;
  }

  var action_id = parseInt($title.text());
  if (action_id === 0) {
    // Start terminal
    callback(0, 'Start');
  } else if (action_id < 0) {
    // End terminal
    callback(action_id);
  } else if ($target.text() === '\u27a6') {
    // Link to sub task template (skip callback)
  } else {
    // Real action
    let outcome_id = $target.parent().parent().attr('id');
    let outcome_hint = `a_action_${action_id}_outcome__`;
    let outcome_key;
    if (outcome_id && outcome_id.indexOf(outcome_hint) === 0) {
      outcome_key = outcome_id.substr(outcome_hint.length);
    }
    callback(action_id, outcome_key);
  }
}

function onClickEdge(evt, edge) {
  var $title = $(edge).find('title');
  if ($title.length !== 1) {
    return;
  }

  var title = $title.text();
  var outcome = title.substr(0, title.lastIndexOf('->')).split(':');
  var action_id = parseInt(outcome.shift());
  if (action_id === 0) {
    callback(0, 'Start');
  } else {
    let outcome_key = outcome.join(':');
    callback(action_id, outcome_key);
  }
}

var _cb;

function setCallback(cb) {
  if (_.isFunction(cb)) {
    _cb = cb;
  } else {
    _cb = null;
  }
}

function callback(action_id, outcome_key) {
  if (_cb) {
    _cb(action_id, outcome_key);
  }
}

export default {
  /* Hit Test */
  bind: bindHitTest,
  unbind: unbindHitTest,
  setCallback: setCallback
};
