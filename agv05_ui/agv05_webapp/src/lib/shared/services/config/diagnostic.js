import { getAPI } from '$lib/utils';

function get(fetch = undefined) {
  return getAPI('/config/diagnostics', fetch);
}

export default {
  get
};
