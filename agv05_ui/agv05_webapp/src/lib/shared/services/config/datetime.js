import { getAPI, postAPI } from '$lib/utils';

function successUpdateMsg() {
  return 'Date Time configuration updated.';
}

function get(fetch = undefined) {
  return getAPI('/config/date-time', fetch);
}

function update(data, fetch = undefined) {
  return postAPI('/config/date-time', data, 'POST', fetch);
}

export default {
  successUpdateMsg,
  get,
  update
};
