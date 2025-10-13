import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ params, fetch }) => {
  await pagePermissions(['system.view_system_panel', 'systemx.change_mapocg']);
  const mapID = params.id;
  const ocgID = params.ocgID;

  const ocg = await maps.ocg(mapID, ocgID, fetch);
  if (!ocg) {
    throw error(404, 'Not Found');
  }
  return {
    mapID,
    ocg
  };
};
