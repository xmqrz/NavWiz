import { pagePermissions } from 'stores/auth.js';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_tasktemplate']);
  const metaFetch = taskTemplates.meta(fetch);
  const globalParamFetch = taskTemplates.getGlobalParam(fetch);

  const [meta, globalParam] = await Promise.all([metaFetch, globalParamFetch]);
  return {
    meta,
    globalParam
  };
};
