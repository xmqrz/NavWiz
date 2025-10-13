import { pagePermissions, noPermissionError } from 'stores/auth.js';
import permissions from '$lib/shared/services/config/permissions';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'auth.change_permission']);

  const permissionsFetch = permissions.get(fetch);
  const optionsFetch = permissions.getOptions(fetch);

  const [permissionsData, options] = await Promise.all([permissionsFetch, optionsFetch]);

  if (!permissionsData || !options || !options.actions || !options.actions.POST) {
    throw noPermissionError();
  }
  try {
    permissionsData.layout = JSON.parse(permissionsData.layout);
  } catch (err) {
    console.log('Fail to load layout in permissions page');
  }

  let properties = {};
  try {
    properties = options.actions.POST;
  } catch (err) {
    console.log('Fail to obtain properties in permissions page');
  }

  return {
    permissions: permissionsData,
    properties
  };
};
