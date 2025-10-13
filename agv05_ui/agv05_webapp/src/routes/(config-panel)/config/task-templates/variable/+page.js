import { pagePermissions } from 'stores/auth.js';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_tasktemplate']);
  const metaFetch = taskTemplates.meta(fetch);
  const variableFetch = taskTemplates.getVariable(fetch);

  const [meta, variable] = await Promise.all([metaFetch, variableFetch]);
  return {
    meta,
    variable
  };
};
