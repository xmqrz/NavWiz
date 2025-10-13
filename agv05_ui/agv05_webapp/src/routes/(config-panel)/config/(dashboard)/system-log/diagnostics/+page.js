import { pagePermissions } from 'stores/auth.js';
import diagnostics from '$lib/shared/services/config/diagnostic';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_log_files']);

  const diagnosticsList = await diagnostics.get(fetch);
  return {
    diagnosticsList: diagnosticsList || []
  };
};
