import { error, redirect } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import taskCompleted from '$lib/shared/services/config/task-completed';

const limit = 100;

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  const page = Number(url.searchParams.get('page')) || 1;

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  await pagePermissions(['system.view_system_panel', 'system.view_completed_tasks']);
  let data = await taskCompleted.getList(page, limit, fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, '/config/task-completed');
  }

  let count = data.count || 0;
  let results = data.results || [];
  let pageCount = data.count ? Math.ceil(data.count / limit) : 1;

  return {
    count,
    completedTasks: results,
    pageCount,
    page,
    limit
  };
};
