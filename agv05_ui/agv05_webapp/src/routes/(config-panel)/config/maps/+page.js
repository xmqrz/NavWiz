import { pagePermissions } from 'stores/auth.js';
import { error, redirect } from '@sveltejs/kit';
import maps from '$lib/shared/services/config/maps';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ url, fetch }) => {
  await pagePermissions(['system.view_system_panel']);
  const limit = 100;

  const page = Number(url.searchParams.get('page')) || 1;

  if (page <= 0) {
    throw error(400, 'Bad Request');
  }

  let listFetch = maps.getList(page, limit, fetch);
  let activeFetch = maps.active(fetch);
  let [data, active] = await Promise.all([listFetch, activeFetch]);

  if (page > 1 && data && data.results && data.results.length === 0) {
    throw redirect(302, maps.listUrl());
  }

  data = data || {};
  active = active ? active.results : [];

  let count = data.count || 0;
  let results = data.results || [];
  let pageCount = data.count ? Math.ceil(data.count / limit) : 1;
  return {
    count,
    maps: results,
    active,
    pageCount,
    page,
    limit
  };
};
