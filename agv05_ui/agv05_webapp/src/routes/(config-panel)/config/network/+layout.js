import { pagePermissions } from 'stores/auth.js';

/** @type {import('@sveltejs/kit').Load} */
export const load = async () => {
  await pagePermissions(['system.view_system_panel']);

  return {};
};
