import { error } from '@sveltejs/kit';

import { pagePermissions } from 'stores/auth.js';
import mapQualityScoreService from '$lib/shared/services/config/mapx-quality';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_map_quality']);

  const mqs = await mapQualityScoreService.get(fetch);
  if (!mqs) {
    throw error(404, 'Not Found');
  }

  return {
    mqs: mqs
  };
};
