import { pagePermissions } from 'stores/auth.js';
import parameters from '$lib/shared/services/config/parameters';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  await pagePermissions(['system.view_system_panel']);

  let listData = await parameters.getList(fetch);

  listData = listData
    ? listData.results
    : {
        active_components: {
          controllers: [],
          hardware: []
        },
        offline_components: {
          controllers: [],
          hardware: []
        }
      };

  return {
    parameters: listData
  };
};
