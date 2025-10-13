import { pagePermissions } from 'stores/auth.js';
import backupRestore from '$lib/shared/services/config/backup-restore';

/** @type {import('@sveltejs/kit').Load} */
export const load = async () => {
  await pagePermissions(['system.view_system_panel', 'system.backup_restore']);
  await backupRestore.checkLicenseVoid(fetch);
  return {};
};
