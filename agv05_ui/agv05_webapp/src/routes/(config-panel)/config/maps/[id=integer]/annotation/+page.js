import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'system.add_mapchangeset']);
  const id = params.id;
  const layoutFetch = maps.layout(id, fetch);
  const annotationFetch = maps.annotation(id, fetch);
  const [layout, annotation] = await Promise.all([layoutFetch, annotationFetch]);
  if (!layout || !annotation) {
    throw error(404, 'Not Found');
  }
  return {
    layout,
    annotation
  };
};
