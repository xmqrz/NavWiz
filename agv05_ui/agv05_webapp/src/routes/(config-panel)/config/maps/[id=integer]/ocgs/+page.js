import { pagePermissions } from 'stores/auth.js';
import { error, redirect } from '@sveltejs/kit';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, params, fetch }) => {
  await pagePermissions(['system.view_system_panel', 'systemx.change_mapocg']);
  const mapID = params.id;
  const limit = 100;

  const page = Number(url.searchParams.get('page')) || 1;

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  let data = await maps.getOcgList(mapID, page, limit, fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, maps.ocgsUrl(mapID));
  }

  data = data || {};

  let count = data.count || 0;
  let results = data.results || [];
  let pageCount = data.count ? Math.ceil(data.count / limit) : 1;
  let mapDisplayName = data.map_display_name || '-';
  return {
    count,
    ocgs: results,
    mapDisplayName,
    mapID,
    pageCount,
    page,
    limit
  };
};
