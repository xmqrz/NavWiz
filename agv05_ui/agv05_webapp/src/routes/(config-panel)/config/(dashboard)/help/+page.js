import { pagePermissions } from 'stores/auth.js';
import license from '$lib/shared/services/config/license';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_help_content']);

  const licenseInfo = await license.info(fetch);
  if (!licenseInfo) {
    throw error(404, 'Not Found');
  }

  return {
    licenseInfo
  };
};
