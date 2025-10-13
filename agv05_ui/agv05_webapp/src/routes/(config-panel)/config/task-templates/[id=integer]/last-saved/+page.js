import { pagePermissions } from 'stores/auth.js';
import taskTemplates from '$lib/shared/services/config/task-templates';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_tasktemplate']);
  const id = params.id;
  const metaFetch = taskTemplates.meta(fetch);
  const taskTemplateFetch = taskTemplates.get(id, fetch);
  const [meta, taskTemplate] = await Promise.all([metaFetch, taskTemplateFetch]);
  if (!taskTemplate) {
    throw error(404, 'Not Found');
  }
  return {
    taskTemplate,
    meta
  };
};
