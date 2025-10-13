/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import utils from './utils';

export default function (viz) {
  function find(id) {
    if (id.startsWith('_Station_')) {
      return findStation(id);
    } else if (id.startsWith('_Register_')) {
      return findRegister(id);
    } else if (id.startsWith('_g_')) {
      return findGlobalParam(id);
    } else if (id.startsWith('_v_')) {
      return findVariable(id);
    } else if (id.startsWith('_ttpk_')) {
      return findTaskTemplate(id);
    } else if (id.startsWith('_a_')) {
      return findTaskTemplate(id.substring(3));
    }
  }

  function findStation(id) {
    let found = [];
    let station = id.substr(9);
    let actions = viz.models.rawActions();

    actions.forEach((action, idx) => {
      if (idx <= 0) {
        return;
      }

      if (utils.findStationInAction(action, station, viz.models)) {
        found.push({
          action_id: idx,
          outcome_key: null
        });
      }
    });

    return found;
  }

  function findRegister(id) {
    let found = [];
    let register = id.substr(10);
    let actions = viz.models.rawActions();

    actions.forEach((action, idx) => {
      if (idx <= 0) {
        return;
      }

      if (utils.findRegisterInAction(action, register, viz.models)) {
        found.push({
          action_id: idx,
          outcome_key: null
        });
      }
    });

    return found;
  }

  function findGlobalParam(id) {
    let name = id.substr(3);
    name = '${' + name + '}g';

    let found = [];
    let actions = viz.models.rawActions();

    actions.forEach((action, idx) => {
      if (idx <= 0) {
        return;
      }

      if (utils.findGlobalParamInAction(action, name, viz.models)) {
        found.push({
          action_id: idx,
          outcome_key: null
        });
      }
    });

    return found;
  }

  function findVariable(id) {
    let name = id.substr(3);

    let found = [];
    let actions = viz.models.rawActions();

    actions.forEach((action, idx) => {
      if (idx <= 0) {
        return;
      }

      if (utils.findVariableInAction(action, name, viz.models)) {
        found.push({
          action_id: idx,
          outcome_key: null
        });
      }
    });

    return found;
  }

  function findTaskTemplate(id) {
    let found = [];
    let actions = viz.models.rawActions();

    actions.forEach((action, idx) => {
      if (idx <= 0) {
        return;
      }
      if (action.skillId !== id) {
        return;
      }
      found.push({
        action_id: idx,
        outcome_key: null
      });
    });

    return found;
  }

  function fillOptions(sel) {
    utils.fillOptions(sel, viz.models);
  }

  viz.search = {
    find: find,
    fillOptions: fillOptions
  };
}
