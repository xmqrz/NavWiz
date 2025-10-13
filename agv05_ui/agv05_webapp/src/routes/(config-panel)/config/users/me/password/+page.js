import { pagePermissions } from 'stores/auth.js';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url }) => {
  await pagePermissions(['system.change_own_password']);
  const next = url.searchParams.get('next');
  return {
    next
  };
};
