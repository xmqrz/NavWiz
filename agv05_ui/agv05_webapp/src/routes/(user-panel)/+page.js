import { pagePermissions } from 'stores/auth.js';

/** @type {import('@sveltejs/kit').Load} */
export const load = async () => {
  await pagePermissions([]); // Required for throwing error if backend unreachable.
  return {};
};
