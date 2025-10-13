import { getAPI, formAPI } from '$lib/utils';

function get(fetch = undefined) {
  return getAPI('/config/hardware-plugin', fetch);
}

function update(data, fetch = undefined) {
  return formAPI('/config/hardware-plugin', data, 'POST', fetch);
}

export default {
  get,
  update
};
