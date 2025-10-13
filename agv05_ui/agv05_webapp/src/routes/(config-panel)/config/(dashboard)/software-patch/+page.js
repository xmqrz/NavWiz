import patch from '$lib/shared/services/config/software-patch';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const data = await patch.get(fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }

  return {
    changelog: data.changelog ?? ''
  };
};
