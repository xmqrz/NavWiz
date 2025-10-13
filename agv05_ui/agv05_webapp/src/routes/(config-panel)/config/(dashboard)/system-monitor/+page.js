import { pagePermissions } from 'stores/auth.js';
import monitor from '$lib/shared/services/config/system-monitor';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_log_files']);
  const data = await monitor.get(fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  return {
    usage: data ?? {}
  };
};
