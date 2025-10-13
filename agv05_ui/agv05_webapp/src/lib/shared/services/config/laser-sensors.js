import { getAPI, postAPI } from '$lib/utils';

// URLS

function url() {
  return '/config/laser-sensors';
}

// MSGS

function successUpdateMsg() {
  // TODO: add translation here..
  return 'Laser sensors updated.';
}

// REST API

function get(fetch = undefined) {
  return getAPI('/config/lasers', fetch);
}

function getOptions(fetch = undefined) {
  return getAPI('/config/lasers', fetch, 'OPTIONS');
}

function update(data, fetch = undefined) {
  return postAPI('/config/lasers', data, 'POST', fetch);
}

export default {
  url,

  successUpdateMsg,

  get,
  getOptions,
  update
};
