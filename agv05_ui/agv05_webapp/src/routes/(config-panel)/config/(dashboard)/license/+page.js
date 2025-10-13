import { pagePermissions } from 'stores/auth.js';
import license from '$lib/shared/services/config/license';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  pagePermissions(['system.view_system_panel', 'system.change_license']);
  const req = await license.req(fetch);
  const licenseInfo = await license.info(fetch);
  if (!req || !licenseInfo) {
    throw error(404, 'Not Found');
  }

  return {
    req,
    licenseInfo
  };
};
