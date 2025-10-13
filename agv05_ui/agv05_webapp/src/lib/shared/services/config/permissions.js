import { getAPI, formAPI, postAPI } from '$lib/utils';

// URLS

function editUrl() {
  return '/config/permissions';
}

// MSGS

function successUpdateMsg() {
  // TODO: add translation here..
  return 'Permissions updated.';
}

function successResetMsg() {
  // TODO: add translation here..
  return `Permissions reset.`;
}

// REST API

function get(fetch = undefined) {
  return getAPI('/config/permissions', fetch);
}

function getOptions(fetch = undefined) {
  return getAPI('/config/permissions', fetch, 'OPTIONS');
}

function update(form, fetch = undefined) {
  return formAPI('/config/permissions', form, 'POST', fetch);
}

function reset(fetch = undefined) {
  return postAPI('/config/permissions/reset', {}, 'POST', fetch);
}

export default {
  editUrl,

  successUpdateMsg,
  successResetMsg,

  get,
  getOptions,
  update,
  reset
};
