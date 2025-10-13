import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'system.add_mapchangeset']);
  const id = params.id;
  const layout = await maps.layout(id, fetch);
  if (!layout) {
    throw error(404, 'Not Found');
  }
  return {
    layout
  };
};
