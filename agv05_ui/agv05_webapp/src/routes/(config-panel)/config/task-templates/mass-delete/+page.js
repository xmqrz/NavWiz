import { pagePermissions } from 'stores/auth.js';
import { error } from '@sveltejs/kit';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, url }) => {
  await pagePermissions(['system.view_system_panel', 'system.delete_tasktemplate']);
  const ttParam = url.searchParams.get('tt');

  if (!ttParam) {
    throw error(400, 'Bad Request');
  }

  const tt = await taskTemplates.getMassDelete(ttParam, fetch);

  return {
    taskTemplates: tt
  };
};
