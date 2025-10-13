import { pagePermissions } from 'stores/auth.js';
import backupRestore from '$lib/shared/services/config/backup-restore';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.backup_restore']);
  const data = await backupRestore.backupAdvancedOptions(fetch);
  const backupChoices = data ? data.backup_choices || {} : {};
  return {
    backupChoices
  };
};
