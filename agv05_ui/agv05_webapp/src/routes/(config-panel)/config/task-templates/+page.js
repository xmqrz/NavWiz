import { pagePermissions } from 'stores/auth.js';
import { error, redirect } from '@sveltejs/kit';
import taskTemplates from '$lib/shared/services/config/task-templates';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  await pagePermissions(['system.view_system_panel']);
  const limit = 100;

  const page = Number(url.searchParams.get('page')) || 1;
  const category = url.searchParams.get('category');
  const is_active = url.searchParams.get('is_active');
  const is_top_level = url.searchParams.get('is_top_level');

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  let data = await taskTemplates.getList(
    page,
    limit,
    category,
    is_active,
    is_top_level,
    fetch
  );

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, taskTemplates.listUrl());
  }

  data = data || {};

  let count = data.count || 0;
  let results = data.results || [];
  let pageCount = data.count ? Math.ceil(data.count / limit) : 1;
  return {
    count,
    taskTemplates: results,
    pageCount,
    page,
    limit,
    filter: {
      category,
      is_active,
      is_top_level
    },
    filterOptions: data.filter_options
  };
};
