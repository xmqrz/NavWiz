import ac from '$lib/shared/services/config/agv-activities';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const data = await ac.getConfig(fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  return {
    configuration: JSON.parse(data.value),
    activities: JSON.parse(data.activities),
    modified: data.modified
  };
};
