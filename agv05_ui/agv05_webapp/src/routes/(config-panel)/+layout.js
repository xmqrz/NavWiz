import { getAPI } from '$lib/utils';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const agvFetch = getAPI('/agv', fetch);
  let [agv] = await Promise.all([agvFetch]);

  agv = agv || {};

  return {
    agv
  };
};
