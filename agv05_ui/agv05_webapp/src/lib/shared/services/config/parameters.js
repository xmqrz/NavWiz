import { getAPI, formAPI, postAPI } from '$lib/utils';

// URLS

function listUrl() {
  return '/config/parameters';
}

function editUrl(key) {
  return `/config/parameters${key}/edit`;
}

// MSGS

function successUpdateMsg(name) {
  // TODO: add translation here..
  return `Parameter component "${name}" updated.`;
}

function successDeleteMsg(name) {
  // TODO: add translation here..
  return `Parameter component "${name}" deleted.`;
}

function successResetMsg(name) {
  // TODO: add translation here..
  return `Parameter component "${name}" reset.`;
}

// REST API

function getList(fetch = undefined) {
  return getAPI(`/config/parameters`, fetch);
}

function get(key, fetch = undefined) {
  return getAPI(`/config/parameters${key}`, fetch);
}

function getOptions(key, allowProtected, fetch = undefined) {
  return getAPI(
    `/config/parameters${key}${allowProtected ? '?protected' : ''}`,
    fetch,
    'OPTIONS'
  );
}

function update(key, allowProtected, form, fetch = undefined) {
  return formAPI(
    `/config/parameters${key}${allowProtected ? '?protected' : ''}`,
    form,
    'PUT',
    fetch
  );
}

function remove(key, fetch = undefined) {
  return postAPI(`/config/parameters${key}`, {}, 'DELETE', fetch, true);
}

function reset(key, fetch = undefined) {
  return postAPI(`/config/parameters${key}/reset`, {}, 'POST', fetch);
}

export default {
  listUrl,
  editUrl,

  successUpdateMsg,
  successDeleteMsg,
  successResetMsg,

  getList,
  get,
  getOptions,
  update,
  remove,
  reset
};
