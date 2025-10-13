import { pagePermissions } from 'stores/auth.js';
import { error, redirect } from '@sveltejs/kit';
import groups from '$lib/shared/services/config/groups';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  await pagePermissions(['system.view_system_panel', 'system.view_users']);
  const limit = 100;

  const page = Number(url.searchParams.get('page')) || 1;

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  let data = await groups.getList(page, limit, fetch);
  let builtinGroups = await groups.builtinGroups(fetch);

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, groups.listUrl());
  }

  data = data || {};
  builtinGroups = builtinGroups || {};

  if (builtinGroups) {
    builtinGroups = builtinGroups.map(function (g) {
      return g.name;
    });
  }

  let count = data.count || 0;
  let results = data.results || [];
  let pageCount = data.count ? Math.ceil(data.count / limit) : 1;
  return {
    count,
    groups: results,
    builtinGroups: builtinGroups,
    pageCount,
    page,
    limit
  };
};
