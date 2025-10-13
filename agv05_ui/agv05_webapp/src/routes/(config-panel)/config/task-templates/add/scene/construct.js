/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';
import * as d3 from 'd3';

import EditMode from './edit-mode.js';

export default function (viz, scene, hitTest) {
  /* Objects under construction */
  var $viz = $(viz.node());
  var Utils = scene.Utils;
  var arrow = scene.overlay.append('g');
  arrow.attr('class', 'arrow');
  hitTest.setCallback(hitTestCallback);

  function hitTestCallback(action_id, outcome_key) {
    if (scene.mode === EditMode.POINTER) {
      if (action_id < 0) {
        scene.activeObject.set(null, null);
      } else {
        scene.activeObject.set(action_id, outcome_key);
      }
    } else if (scene.mode === EditMode.LINK) {
      if (action_id !== null && outcome_key) {
        scene.activeObject.set(action_id, outcome_key);
      } else if (action_id !== null) {
        var ob = scene.activeObject.get();
        if (ob.action_id !== null && ob.outcome_key) {
          viz.models.updateActionOutcome(ob.action_id, ob.outcome_key, action_id);
          scene.mode = EditMode.POINTER;
          $viz.trigger('scene.editMode');
          $viz.trigger('scene.activeObject');
        } else {
          scene.activeObject.set(null, null);
        }
      }
    }
  }

  var track = d3
    .track(true)
    .on('trackstart', trackstarted)
    .on('track', tracked)
    .on('trackend', trackended);

  function trackstarted(event) {
    /* jshint validthis: true */
    let ob = Utils.findOutcome(event.trackTarget);
    if (!ob) {
      this._outcome = undefined;
      return;
    }

    event.stopPropagation();
    this._outcome = ob;
  }

  function tracked(event) {
    if (!event) {
      return;
    }
    /* jshint validthis: true */
    if (!this._outcome) {
      return;
    }

    arrow.selectAll('g').remove();
    let ob = Utils.findNode(event.trackTarget);
    if (!ob) {
      this._node = undefined;
      return;
    }
    this._node = ob;

    let srcRect = this._outcome.elem.getBBox();
    let dstRect = this._node.elem.getBBox();
    let g = arrow.append('g');
    Utils.appendConnectorArrow(g, srcRect, dstRect);
  }

  function trackended() {
    /* jshint validthis: true */
    tracked.call(this);
    arrow.selectAll('g').remove();
    if (this._node && this._outcome) {
      scene.activeObject.set(this._outcome.action_id, this._outcome.outcome_key);
      viz.models.updateActionOutcome(
        this._outcome.action_id,
        this._outcome.outcome_key,
        this._node.action_id
      );

      scene.mode = EditMode.POINTER;
      $viz.trigger('scene.editMode');
      $viz.trigger('scene.activeObject');
    }
    this._node = undefined;
    this._outcome = undefined;
  }

  function setEditMode(mode) {
    if (mode === EditMode.LINK) {
      scene.activeObject.set(null, null);
    }
  }

  scene.call(track);

  return {
    setEditMode: setEditMode
  };
}
