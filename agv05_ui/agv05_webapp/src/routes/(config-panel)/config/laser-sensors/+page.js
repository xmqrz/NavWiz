import * as _ from 'lodash-es';

import { pagePermissions, noPermissionError } from 'stores/auth.js';
import laserSensors from '$lib/shared/services/config/laser-sensors';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_parameter']);
  const lasersFetch = laserSensors.get(fetch);
  const optionsFetch = laserSensors.getOptions(fetch);
  const [lasers, options] = await Promise.all([lasersFetch, optionsFetch]);

  if (!options || !options.actions || !options.actions.POST) {
    throw noPermissionError();
  }

  let properties = {};
  try {
    properties = options.actions.POST;
  } catch (err) {
    console.log('Fail to obtain properties in laser page');
  }

  return {
    lasers,
    properties
  };
};
