import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import agvActivities from '$lib/shared/services/config/agv-activities';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params, url }) => {
  await pagePermissions(['system.view_system_panel', 'system.change_agvactivity']);
  const id = params.id;
  const activityFetch = agvActivities.get(id, fetch);
  const activityChoicesFetch = agvActivities.getActivityChoices(fetch);
  let [activity, activityChoices] = await Promise.all([activityFetch, activityChoicesFetch]);
  if (!activity) {
    throw error(404, 'Not Found');
  }
  if (activityChoices) {
    activityChoices = JSON.parse(activityChoices);
  }
  const next = url.searchParams.get('next');
  return {
    activity,
    activityChoices,
    next
  };
};
