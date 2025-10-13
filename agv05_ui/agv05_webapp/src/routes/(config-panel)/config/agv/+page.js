import { pagePermissions, getEnv } from 'stores/auth.js';
import agv from '$lib/shared/services/config/agv';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_agv']);

  let data = await agv.get(fetch);

  data = data || {};

  var allowedMotionsChoices = [
    { value: 'omni', name: 'Omni' },
    { value: 'forward', name: 'Forward' },
    { value: 'reverse', name: 'Reverse' },
    { value: 'rotate_left', name: 'Rotate Left' },
    { value: 'rotate_right', name: 'Rotate Right' },
    { value: 'uturn_left', name: 'U-turn Left, Rotate Left at Destination' },
    { value: 'uturn_right', name: 'U-turn Right, Rotate Right at Destination' },
    { value: 'dynamic', name: 'Dynamic Path Planning' }
  ];

  // Hide `omni_drive` option if the feature is not enabled
  if (!getEnv('TRACKLESS') || !getEnv('OMNI_DRIVE')) {
    allowedMotionsChoices.shift();
  }

  // Hide `dynamic_path_planning` option if the feature is not enabled
  if (!getEnv('DYNAMIC_PATH_PLANNING')) {
    allowedMotionsChoices.pop();
  }

  // initialize model with default value.
  data.agv_name = data.agv_name || '';
  data.executor_mode = data.executor_mode || 1;
  data.fms_endpoint = data.fms_endpoint || '';
  data.agv_home = data.agv_home || '';
  data.allowed_motions = data.allowed_motions || allowedMotionsChoices.map((o) => o.value);
  data.custom_init = data.custom_init || [-1];
  data.station_init = data.station_init || 1;
  data.station_init_stations = data.station_init_stations || [];
  data.pre_init = data.pre_init || [];
  data.default_init = data.default_init || '';
  data.default_init_timeout = data.default_init_timeout || 5;
  data.default_paused = data.default_paused || false;
  data.default_app = data.default_app || '';
  data.default_app_timeout = data.default_app_timeout || 5;
  data.min_battery_level = data.min_battery_level || 30;

  let initChoices = JSON.parse(data.init_choices || '[]');
  let preInitChoices = JSON.parse(data.pre_init_choices || '[]');
  let stationInitChoices = JSON.parse(data.station_init_choices || '[]');
  let stationChoices = JSON.parse(data.station_choices || '[]');

  return {
    agv: data,
    allowedMotionsChoices,
    initChoices,
    preInitChoices,
    stationInitChoices,
    stationChoices
  };
};
