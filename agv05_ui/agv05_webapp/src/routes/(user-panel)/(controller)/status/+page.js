import { pagePermissions } from 'stores/auth.js';

/** @type {import('@sveltejs/kit').Load} */
export const load = async () => {
  await pagePermissions(['system.view_panel', 'app.show_panel_health_status']);

  return {};
};
