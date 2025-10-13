/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

export default function (viz, _scene) {
  var $viz = $(viz.node());
  var ob = {
    action_id: null,
    outcome_key: null
  };

  function getActiveObject() {
    return ob;
  }

  function setActiveObject(action_id, outcome_key, focus) {
    if (ob.action_id !== action_id || ob.outcome_key !== outcome_key) {
      ob = {
        action_id: action_id,
        outcome_key: outcome_key
      };
      viz.modelsUpdated();
      $viz.trigger('scene.activeObject');

      if (focus && action_id > 0) {
        let n = viz.scene.select(`#node${action_id + 1}`);
        if (n) {
          let bbox = n.node().getBBox();
          let sbbox = viz.scene.node().getBBox();
          let gbbox = viz.scene.node().firstChild.getBBox();
          let vbbox = viz.node().getBoundingClientRect();
          let x = -bbox.x - bbox.width / 2 + sbbox.x + gbbox.x + vbbox.width / 2;
          let y = -bbox.y - bbox.height / 2 + sbbox.y + gbbox.y + vbbox.height / 4;
          viz.zoom.zoomCenter(x, y);
        }
      }
    }
  }

  function remove() {
    if (ob.action_id !== null && ob.outcome_key) {
      viz.models.removeActionOutcome(ob.action_id, ob.outcome_key);
      setActiveObject(null);
    } else if (ob.action_id !== null) {
      viz.models.removeAction(ob.action_id);
      setActiveObject(null);
    }
  }

  function isStartTerminal() {
    return ob.action_id === 0;
  }

  function isEndTerminal() {
    return ob.action_id < 0;
  }

  function isAction() {
    return !isStartTerminal() && !isEndTerminal() && ob.action_id && !ob.outcome_key;
  }

  function isActionOutcome() {
    return !isStartTerminal() && !isEndTerminal() && ob.action_id && ob.outcome_key;
  }

  return {
    get: getActiveObject,
    set: setActiveObject,
    remove: remove,
    isStartTerminal: isStartTerminal,
    isEndTerminal: isEndTerminal,
    isAction: isAction,
    isActionOutcome: isActionOutcome
  };
}
