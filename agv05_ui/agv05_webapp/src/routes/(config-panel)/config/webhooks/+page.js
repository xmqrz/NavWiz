import { pagePermissions } from 'stores/auth.js';
import { error, redirect } from '@sveltejs/kit';
import webhooks from '$lib/shared/services/config/webhooks';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  await pagePermissions(['system.view_system_panel']);
  const limit = 100;

  const page = Number(url.searchParams.get('page')) || 1;

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  let data = await webhooks.getList(page, limit, fetch);

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, webhooks.listUrl());
  }

  data = data || {};

  let count = data.count || 0;
  let results = data.results || [];
  let pages = data.count ? Math.ceil(data.count / limit) : 1;
  return {
    count,
    items: results,
    pages,
    page,
    limit
  };
};
