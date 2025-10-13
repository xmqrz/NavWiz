import { pagePermissions } from 'stores/auth.js';
import { error } from '@sveltejs/kit';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, url }) => {
  await pagePermissions([
    'system.view_system_panel',
    'system.change_tasktemplate',
    'system.view_users'
  ]);
  const ttParam = url.searchParams.get('tt');

  if (!ttParam) {
    throw error(400, 'Bad Request');
  }

  const data = await taskTemplates.getMassUsersUpdate(ttParam, fetch);

  return {
    taskTemplates: data.task_templates,
    availableUsers: data.available_users,
    availableGroups: data.available_groups
  };
};
