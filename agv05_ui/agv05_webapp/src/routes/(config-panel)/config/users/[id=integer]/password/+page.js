import { error } from '@sveltejs/kit';

import { pagePermissions } from 'stores/auth.js';
import users from '$lib/shared/services/config/users';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'auth.change_user']);
  const id = params.id;
  const user = await users.get(id, fetch);
  if (!user) {
    throw error(404, 'Not Found');
  }
  return {
    user
  };
};
