import { pagePermissions } from 'stores/auth.js';
import { redirect } from '@sveltejs/kit';
import maps from '$lib/shared/services/config/maps';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_map']);
  const tTriggerFetch = maps.tTrigger(fetch);
  const metaFetch = taskTemplates.meta(fetch);
  const [tTrigger, meta] = await Promise.all([tTriggerFetch, metaFetch]);

  if (!tTrigger) {
    throw redirect(302, maps.listUrl());
  }

  return {
    tTrigger,
    meta
  };
};
