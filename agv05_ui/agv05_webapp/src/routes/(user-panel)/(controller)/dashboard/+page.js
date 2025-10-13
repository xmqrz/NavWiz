import { pagePermissions } from 'stores/auth.js';
import { getAPI } from '$lib/utils';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_panel']);
  const dashboard = getAPI('/dashboard/entry', fetch);

  return dashboard || {};
};
