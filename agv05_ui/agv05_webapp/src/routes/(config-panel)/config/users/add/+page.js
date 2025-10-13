import { pagePermissions } from 'stores/auth.js';
import users from '$lib/shared/services/config/users';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'auth.add_user']);
  const availableGroupsFetch = users.availableGroups(fetch);
  const [availableGroups] = await Promise.all([availableGroupsFetch]);
  if (!availableGroups) {
    throw error(404, 'Not Found');
  }
  return {
    availableGroups
  };
};
