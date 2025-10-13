import { pagePermissions } from 'stores/auth.js';
import { getAPI } from '$lib/utils';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel']);

  const dashboardFetch = getAPI('/dashboard', fetch);
  const agvFetch = getAPI('/agv', fetch);
  let [dashboard, agv] = await Promise.all([dashboardFetch, agvFetch]);

  agv = agv || {};
  // cleanup data.
  if (agv.mileage) {
    agv.mileage = agv.mileage.toFixed(2);
  }

  return {
    dashboard: dashboard || {},
    agv
  };
};
