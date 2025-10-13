import { getAPI, formAPI } from '$lib/utils';

// URLS

function url() {
  return '/config/audio';
}

// MSGS

function successUpdateMsg() {
  // TODO: add translation here..
  return 'Audio updated.';
}

// REST API

function get(fetch = undefined) {
  return getAPI('/config/audio', fetch);
}

function getOptions(fetch = undefined) {
  return getAPI('/config/audio', fetch, 'OPTIONS');
}

function update(form, fetch = undefined) {
  return formAPI('/config/audio', form, 'POST', fetch);
}

export default {
  url,

  successUpdateMsg,

  get,
  getOptions,
  update
};
