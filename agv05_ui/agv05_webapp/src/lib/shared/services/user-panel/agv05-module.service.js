import { elevatedPostAPI as postAPI, getAPI } from '$lib/utils';

export const agv05Module = {
  startModule: function (module_id) {
    return postAPI('/apps', {
      operation: 1,
      app_id: module_id
    });
  },
  stopModule: function (module_id) {
    return postAPI('/apps', {
      operation: 0,
      app_id: module_id
    });
  },
  getModule: function (module_id) {
    if (module_id) {
      return getAPI('/apps/' + module_id);
    }
    return getAPI('/apps');
  }
};
