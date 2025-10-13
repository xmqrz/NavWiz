import { getAPI, formAPI } from '$lib/utils';

// URLS

function editUrl() {
  return '/config/white-label';
}

// MSGS

function successUpdateMsg() {
  // TODO: add translation here..
  return 'White label settings updated.';
}

// REST API

function get(fetch = undefined) {
  return getAPI('/white-label/update-label', fetch);
}

function getOptions(fetch = undefined) {
  return getAPI('/white-label/update-label', fetch, 'OPTIONS');
}

function update(form, fetch = undefined) {
  return formAPI('/white-label/update-label', form, 'POST', fetch);
}

export default {
  editUrl,

  successUpdateMsg,

  get,
  getOptions,
  update
};
