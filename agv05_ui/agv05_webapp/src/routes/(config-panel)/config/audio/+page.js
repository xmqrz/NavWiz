import { pagePermissions, noPermissionError } from 'stores/auth.js';
import audio from '$lib/shared/services/config/audio';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_parameter']);
  const parameterFetch = await audio.get(fetch);
  const optionsFetch = await audio.getOptions(fetch);
  const [parameter, options] = await Promise.all([parameterFetch, optionsFetch]);
  if (!parameter || !options || !options.actions || !options.actions.POST) {
    throw noPermissionError();
  }
  try {
    parameter.layout = JSON.parse(parameter.layout);
  } catch (err) {
    console.log('Fail to load layout in audio config page');
  }

  let properties = {};
  try {
    properties = options.actions.POST;
  } catch (err) {
    console.log('Fail to obtain properties in audio config page');
  }

  return {
    parameter,
    properties
  };
};
