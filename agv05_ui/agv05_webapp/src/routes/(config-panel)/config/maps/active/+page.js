import { pagePermissions } from 'stores/auth.js';
import { redirect } from '@sveltejs/kit';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_map']);
  let active = await maps.active(fetch);

  if (!active) {
    throw redirect(302, maps.listUrl());
  }

  return {
    active
  };
};
