import { getAPI } from '$lib/utils';

function get(fetch = undefined) {
  return getAPI('/config/system-monitor', fetch);
}

export default {
  get
};
