import { pagePermissions, noPermissionError } from 'stores/auth.js';
import ioService from '$lib/shared/services/config/io';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel']);
  const io = await ioService.get(fetch);
  if (!io) {
    throw noPermissionError();
  }

  return {
    io
  };
};
