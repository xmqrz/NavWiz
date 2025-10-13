import plugin from '$lib/shared/services/config/hardware-plugin';
import { error } from '@sveltejs/kit';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const data = await plugin.get(fetch);
  if (!data || !Array.isArray(data.plugins)) {
    throw error(404, 'Not Found');
  }

  return {
    plugins: data.plugins
  };
};
