import * as _ from 'lodash-es';

import { pagePermissions } from 'stores/auth.js';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, url }) => {
  await pagePermissions(['system.view_system_panel', 'system.add_tasktemplate']);
  const from = url.searchParams.get('from');
  const metaFetch = taskTemplates.meta(fetch);
  let taskTemplateFetch = new Promise((resolve) =>
    resolve({
      is_active: true,
      is_top_level: true
    })
  );
  if (from) {
    try {
      const id = parseInt(from);
      taskTemplateFetch = taskTemplates.get(id, fetch);
    } catch (e) {
      console.log(`Fail to fetch task template with id "${from}"`);
    }
  }

  const [meta, taskTemplate] = await Promise.all([metaFetch, taskTemplateFetch]);
  return {
    meta,
    taskTemplate: _.pick(taskTemplate, [
      'metadata',
      'structure',
      'category',
      'is_top_level',
      'is_active'
    ])
  };
};
