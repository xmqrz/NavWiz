/*
 * Copyright (c) 2022, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

function fillOptions(selectField, models) {
  let group;
  let options = [];

  if (models.skillDescriptions) {
    group = {
      name: 'Built-in Action',
      options: []
    };
    var skillDescriptions = models.skillDescriptions();
    for (let key in skillDescriptions) {
      let skill = skillDescriptions[key];
      group.options.push([`_a_${skill.id}`, skill.name]);
    }
    options.push(group);
  }

  if (models.taskTemplateMetas) {
    group = {
      name: 'Task Template',
      options: []
    };
    var taskTemplateMetas = models.taskTemplateMetas();
    for (let key in taskTemplateMetas) {
      let skill = taskTemplateMetas[key];
      group.options.push([skill.id, skill.name]);
    }
    options.push(group);
  }

  if (models.globalParams) {
    group = {
      name: 'Global Parameter',
      options: []
    };
    var globalParams = models.globalParams();
    for (let p of globalParams) {
      group.options.push([`_g_${p.name}`, p.name]);
    }
    options.push(group);
  }

  if (models.variables) {
    group = {
      name: 'Variable',
      options: []
    };
    var variables = models.variables();
    for (let v of variables) {
      group.options.push([`_v_${v.name}`, v.name]);
    }
    options.push(group);
  }

  if (models.stationList) {
    group = {
      name: 'Station',
      options: []
    };
    var stationList = models.stationList();
    for (let s of stationList) {
      group.options.push([`_Station_${s}`, s]);
    }
    options.push(group);
  }

  if (models.registerList) {
    group = {
      name: 'Register',
      options: []
    };
    var registerList = models.registerList();
    for (let r of registerList) {
      group.options.push([`_Register_${r}`, r]);
    }
    options.push(group);
  }

  selectField.options = options;
}

function getActionSkillDesc(action, models) {
  if (!action || !action.skillId) {
    return;
  }
  if (action.skillId.startsWith('_ttpk_')) {
    return models.taskTemplateMetas()[action.skillId];
  } else {
    return models.skillDescriptions()[action.skillId];
  }
}

function findStationInAction(action, station, models) {
  let skillDesc = getActionSkillDesc(action, models);
  if (!skillDesc) {
    return;
  }

  for (let p of skillDesc.params) {
    if (p.type !== 'Station') {
      continue;
    }
    let val = action.params[p.key];
    if (val !== station) {
      continue;
    }
    return true;
  }
}

function findRegisterInAction(action, register, models) {
  let skillDesc = getActionSkillDesc(action, models);
  if (!skillDesc) {
    return;
  }

  for (let p of skillDesc.params) {
    if (p.type !== 'Register') {
      continue;
    }
    let val = action.params[p.key];
    if (val !== register) {
      continue;
    }
    return true;
  }
}

function findGlobalParamInAction(action, gpInherit, models) {
  let skillDesc = getActionSkillDesc(action, models);
  if (!skillDesc) {
    return;
  }

  for (let p of skillDesc.params) {
    let val = action.params[p.key];
    if (val !== gpInherit) {
      continue;
    }
    return true;
  }
}

function findVariableInAction(action, variable, models) {
  let skillDesc = getActionSkillDesc(action, models);
  if (!skillDesc) {
    return;
  }

  let inheritName = '${' + variable + '}v';

  for (let p of skillDesc.params) {
    let val = action.params[p.key];
    if (val !== inheritName && !p.type.startsWith('v')) {
      continue;
    } else if (p.type.startsWith('v') && val !== variable) {
      continue;
    }
    return true;
  }
}

export default {
  fillOptions: fillOptions,
  findStationInAction: findStationInAction,
  findRegisterInAction: findRegisterInAction,
  findGlobalParamInAction: findGlobalParamInAction,
  findVariableInAction: findVariableInAction
};
