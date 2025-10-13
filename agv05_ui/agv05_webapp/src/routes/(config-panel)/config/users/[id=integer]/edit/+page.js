import { pagePermissions } from 'stores/auth.js';
import users from '$lib/shared/services/config/users';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'auth.change_user']);
  const id = params.id;
  const userFetch = users.get(id, fetch);
  const availableGroupsFetch = users.availableGroups(fetch);
  const [user, availableGroups] = await Promise.all([userFetch, availableGroupsFetch]);
  if (!user || !availableGroups) {
    throw error(404, 'Not Found');
  }
  return {
    user,
    availableGroups
  };
};
