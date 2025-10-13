import { pagePermissions } from 'stores/auth.js';
import users from '$lib/shared/services/config/users';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, url }) => {
  await pagePermissions(['system.change_own_password']);
  const next = url.searchParams.get('next');
  const authToken = await users.ownAuthToken(fetch);
  if (!authToken) {
    throw error(404, 'Not Found');
  }
  return {
    authToken,
    next
  };
};
