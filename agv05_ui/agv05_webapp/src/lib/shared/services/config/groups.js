import { getAPI, postAPI } from '$lib/utils';

// URLS

function listUrl() {
  return '/config/groups';
}

function addUrl() {
  return '/config/groups/add';
}

function editUrl(id) {
  return `/config/groups/${id}/edit`;
}

// MSGS

function successAddMsg(name) {
  // TODO: add translation here..
  return `Group "${name}" created.`;
}

function successUpdateMsg(name) {
  // TODO: add translation here..
  return `Group "${name}" updated.`;
}

function successDeleteMsg(name) {
  // TODO: add translation here..
  return `Group "${name}" deleted.`;
}

// REST API
function builtinGroups(fetch = undefined) {
  return getAPI('/config/groups/builtin-groups', fetch);
}

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/groups?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/groups/${id}`, fetch);
}

function add(data, fetch = undefined) {
  return postAPI(`/config/groups`, data, 'POST', fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/groups/${id}`, data, 'PATCH', fetch);
}

function remove(id, fetch = undefined) {
  return postAPI(`/config/groups/${id}`, {}, 'DELETE', fetch, true);
}

export default {
  listUrl,
  addUrl,
  editUrl,
  successAddMsg,
  successUpdateMsg,
  successDeleteMsg,

  builtinGroups,
  getList,
  get,
  add,
  update,
  remove
};
