import { pagePermissions, noPermissionError } from 'stores/auth.js';
import whiteLabel from '$lib/shared/services/config/white-label';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_white_label']);

  const permissionsFetch = whiteLabel.get(fetch);
  const optionsFetch = whiteLabel.getOptions(fetch);

  const [whiteLabelData, options] = await Promise.all([permissionsFetch, optionsFetch]);

  if (!whiteLabelData || !options || !options.actions || !options.actions.POST) {
    throw noPermissionError();
  }

  let properties = {};
  try {
    properties = options.actions.POST;
  } catch (err) {
    console.log('Fail to obtain properties in white label page');
  }

  return {
    whiteLabel: whiteLabelData,
    properties
  };
};
