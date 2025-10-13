import { getAPI, postAPI } from '$lib/utils';

function successUpdateMsg() {
  return 'IO configuration updated.';
}

function get(fetch = undefined) {
  return getAPI('/config/io', fetch);
}

function update(data, fetch = undefined) {
  return postAPI('/config/io', data, 'POST', fetch);
}

export default {
  successUpdateMsg,
  get,
  update
};
