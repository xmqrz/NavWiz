import { pagePermissions } from 'stores/auth.js';
import groups from '$lib/shared/services/config/groups';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'system.view_users', 'auth.change_user']);
  const id = params.id;
  const group = await groups.get(id, fetch);
  if (!group) {
    throw error(404, 'Not Found');
  }
  return {
    group
  };
};
