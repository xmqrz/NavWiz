import { error } from '@sveltejs/kit';

import { pagePermissions } from 'stores/auth.js';
import parameters from '$lib/shared/services/config/parameters';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params, url }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_parameter']);
  const allowProtected = url.searchParams.has('protected');
  const key = '/' + params.key;
  const parameterFetch = parameters.get(key, fetch);
  const optionsFetch = parameters.getOptions(key, allowProtected, fetch);
  const [parameter, options] = await Promise.all([parameterFetch, optionsFetch]);
  if (!parameter || !options || !options.actions || !options.actions.PUT) {
    throw error(404, 'Not Found');
  }
  try {
    parameter.layout = JSON.parse(parameter.layout);
  } catch (err) {
    console.log('Fail to load layout in parameter page');
  }

  let properties = {};
  try {
    properties = options.actions.PUT;
  } catch (err) {
    console.log('Fail to obtain properties in parameter page');
  }

  return {
    parameter,
    allowProtected,
    properties
  };
};
