import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import datetimeService from '$lib/shared/services/config/datetime';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.change_datetime']);
  const datetime = await datetimeService.get(fetch);
  if (!datetime) {
    throw error(404, 'Not Found');
  }

  return {
    datetime
  };
};
