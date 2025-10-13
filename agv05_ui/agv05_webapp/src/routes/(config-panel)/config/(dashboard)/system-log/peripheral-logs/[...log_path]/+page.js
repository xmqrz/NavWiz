import { pagePermissions } from 'stores/auth.js';
import log from '$lib/shared/services/config/log';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_log_files']);

  const logPath = params.log_path;
  let data = await log.getPeripheralLog(logPath, fetch);
  data = data || {};
  data.logPath = logPath;
  return data;
};
