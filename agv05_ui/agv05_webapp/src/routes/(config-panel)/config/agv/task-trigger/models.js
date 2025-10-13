/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as _ from 'lodash-es';

import Action from 'task-template-editor/models/action';
import TaskTemplateMeta from '$lib/shared/models/task-template-meta';

export default function (form) {
  var taskTriggers = form.taskTriggers;
  var taskTemplateMetas = {};
  var stationList = [];
  var registerList = [];

  // field data
  var agvIdle = {
    timeout: 3,
    abortable: false,
    ignore_charging: false,
    ignore_home: false,
    ignore_stations: [],
    ignore_low_battery: false,
    ignore_trigger_active: false,
    action: new Action()
  };

  var batteryLow = {
    threshold: 0,
    abortable: false,
    ignore_charging: false,
    ignore_home: false,
    ignore_stations: [],
    ignore_low_battery: false,
    ignore_trigger_active: false,
    action: new Action()
  };

  /* Deserialize from database */
  function load(options) {
    if (options.taskTemplateMetas && Array.isArray(options.taskTemplateMetas)) {
      taskTemplateMetas = _.keyBy(
        _.map(options.taskTemplateMetas, (t) => new TaskTemplateMeta(t)).filter(
          (tt) => tt.is_active && tt.is_top_level
        ),
        'id'
      );
    }
    if (options.stationList && Array.isArray(options.stationList)) {
      stationList = options.stationList;
    }
    if (options.registerList && Array.isArray(options.registerList)) {
      registerList = options.registerList;
    }
    if (options.taskTriggers) {
      // load agv idle row
      let agv_idle = options.taskTriggers.agv_idle;
      if (agv_idle) {
        agvIdle.timeout = agv_idle.timeout || agvIdle.timeout;
        agvIdle.abortable = agv_idle.abortable || agvIdle.abortable;
        agvIdle.ignore_charging = agv_idle.ignore_charging || agvIdle.ignore_charging;
        agvIdle.ignore_home = agv_idle.ignore_home || agvIdle.ignore_home;
        agvIdle.ignore_stations = agv_idle.ignore_stations || agvIdle.ignore_stations;
        agvIdle.ignore_low_battery = agv_idle.ignore_low_battery || agvIdle.ignore_low_battery;
        agvIdle.ignore_trigger_active =
          agv_idle.ignore_trigger_active || agvIdle.ignore_trigger_active;
        agvIdle.action = agv_idle.action ? new Action(agv_idle.action) : agvIdle.action;
      }
      // filter missing stations
      if (agvIdle.ignore_stations && Array.isArray(agvIdle.ignore_stations)) {
        agvIdle.ignore_stations = agvIdle.ignore_stations.filter((s) =>
          stationList.includes(s)
        );
      }
      // load battery low row
      let battery_low = options.taskTriggers.battery_low;
      if (battery_low) {
        batteryLow.threshold = battery_low.threshold || batteryLow.threshold;
        batteryLow.abortable = battery_low.abortable || batteryLow.abortable;
        batteryLow.ignore_charging = battery_low.ignore_charging || batteryLow.ignore_charging;
        batteryLow.ignore_home = battery_low.ignore_home || batteryLow.ignore_home;
        batteryLow.ignore_stations = battery_low.ignore_stations || batteryLow.ignore_stations;
        batteryLow.ignore_low_battery =
          battery_low.ignore_low_battery || batteryLow.ignore_low_battery;
        batteryLow.ignore_trigger_active =
          battery_low.ignore_trigger_active || batteryLow.ignore_trigger_active;
        batteryLow.action = battery_low.action
          ? new Action(battery_low.action)
          : batteryLow.action;
      }
      // filter missing stations
      if (batteryLow.ignore_stations && Array.isArray(batteryLow.ignore_stations)) {
        batteryLow.ignore_stations = batteryLow.ignore_stations.filter((s) =>
          stationList.includes(s)
        );
      }
    }

    taskTriggers.trigger('models.loaded');
  }

  function getTaskTriggers() {
    let agvIdleStage = Object.assign({}, agvIdle);
    agvIdleStage.action = agvIdleStage.action.__pick__();
    let batteryLowStage = Object.assign({}, batteryLow);
    batteryLowStage.action = batteryLowStage.action.__pick__();
    return {
      agv_idle: agvIdleStage,
      battery_low: batteryLowStage
    };
  }

  taskTriggers.models = {
    load: load,
    getTaskTriggers: getTaskTriggers,

    agvIdle: () => agvIdle,
    batteryLow: () => batteryLow,
    taskTemplateMetas: () => taskTemplateMetas,
    stationList: () => stationList,
    registerList: () => registerList
  };
}
