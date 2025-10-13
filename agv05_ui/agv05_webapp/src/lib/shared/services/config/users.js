import { getAPI, postAPI } from '$lib/utils';

// URLS

function listUrl() {
  return '/config/users';
}

function addUrl() {
  return '/config/users/add';
}

function editUrl(id) {
  return `/config/users/${id}/edit`;
}

function changePasswordUrl(id) {
  return `/config/users/${id}/password`;
}

function authTokenUrl(id) {
  return `/config/users/${id}/auth-token`;
}

function changeOwnPasswordUrl(next) {
  return `/config/users/me/password${next ? '?next=' + next : ''}`;
}

function myAuthTokenUrl(next) {
  return `/config/users/me/auth-token${next ? '?next=' + next : ''}`;
}

function panelPinUrl() {
  return `/config/users/panel/pin`;
}

// MSGS

function successAddMsg(name) {
  // TODO: add translation here..
  return `User "${name}" created.`;
}

function successUpdateMsg(name) {
  // TODO: add translation here..
  return `User "${name}" updated.`;
}

function successDeleteMsg(name) {
  // TODO: add translation here..
  return `User "${name}" deleted.`;
}

// REST API

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/users?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/users/${id}`, fetch);
}

function authToken(id, fetch = undefined) {
  return getAPI(`/config/users/${id}/auth-token`, fetch);
}

function ownAuthToken(fetch = undefined) {
  return getAPI('/config/users/auth-token', fetch);
}

function availableGroups(fetch = undefined) {
  return getAPI(`/config/users/available-groups`, fetch);
}

function add(data, fetch = undefined) {
  return postAPI(`/config/users`, data, 'POST', fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/users/${id}`, data, 'PUT', fetch);
}

function changePassword(id, data, fetch = undefined) {
  return postAPI(`/config/users/${id}/change-password`, data, 'POST', fetch);
}

function changeOwnPassword(data, fetch = undefined) {
  return postAPI('/config/users/change-password', data, 'POST', fetch);
}

function changeProtectionPin(data, fetch = undefined) {
  return postAPI(`/config/users/change-protection-pin`, data, 'POST', fetch);
}

function regenerateAuthToken(id, fetch = undefined) {
  return postAPI(`/config/users/${id}/auth-token`, {}, 'POST', fetch);
}

function regenerateOwnAuthToken(fetch = undefined) {
  return postAPI('/config/users/auth-token', {}, 'POST', fetch);
}

function remove(id, fetch = undefined) {
  return postAPI(`/config/users/${id}`, {}, 'DELETE', fetch, true);
}

export default {
  listUrl,
  addUrl,
  editUrl,
  changePasswordUrl,
  authTokenUrl,
  changeOwnPasswordUrl,
  myAuthTokenUrl,
  panelPinUrl,
  successAddMsg,
  successUpdateMsg,
  successDeleteMsg,

  getList,
  get,
  authToken,
  ownAuthToken,
  availableGroups,
  add,
  update,
  changePassword,
  changeOwnPassword,
  changeProtectionPin,
  regenerateAuthToken,
  regenerateOwnAuthToken,
  remove
};
