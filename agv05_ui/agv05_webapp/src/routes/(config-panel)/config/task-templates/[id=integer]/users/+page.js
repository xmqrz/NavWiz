import { pagePermissions } from 'stores/auth.js';
import taskTemplates from '$lib/shared/services/config/task-templates';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions([
    'system.view_system_panel',
    'system.change_tasktemplate',
    'system.view_users'
  ]);
  const id = params.id;
  const taskTemplate = await taskTemplates.getUsers(id, fetch);
  if (!taskTemplate) {
    throw error(404, 'Not Found');
  }
  return {
    taskTemplate
  };
};
