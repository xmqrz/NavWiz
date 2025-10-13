import { getAPI } from '$lib/utils';

function get(fetch = undefined) {
  return getAPI('/config/map-quality', fetch);
}

export default {
  get
};
