import { pagePermissions } from 'stores/auth.js';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.add_map']);
  const from = url.searchParams.get('from');
  let map;
  if (from) {
    try {
      const id = parseInt(from);
      map = await maps.get(id, fetch);
    } catch (e) {
      console.log(`Fail to fetch task template with id "${from}"`);
      map = undefined;
    }
  }
  return {
    map
  };
};
