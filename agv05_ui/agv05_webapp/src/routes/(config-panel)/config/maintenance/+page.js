import { error } from '@sveltejs/kit';

import maint from '$lib/shared/services/config/maintenance';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const data = maint.get(fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  return {
    data
  };
};
