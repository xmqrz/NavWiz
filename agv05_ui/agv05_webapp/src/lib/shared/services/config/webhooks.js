import { getAPI, postAPI } from '$lib/utils';

// URLS

function listUrl(page = 1) {
  return `/config/webhooks${page === 1 ? '' : '?page=' + page}`;
}

function addUrl() {
  return '/config/webhooks/add';
}

function editUrl(id) {
  return `/config/webhooks/${id}`;
}

// MSGS

function successAddMsg(url) {
  return `Webhook "${url}" created.`;
}

function successUpdateMsg(url) {
  return `Webhook "${url}" updated.`;
}

function successDeleteMsg(url) {
  return `Webhook "${url}" deleted.`;
}

// REST API

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/webhooks?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/webhooks/${id}`, fetch);
}

function add(data, fetch = undefined) {
  return postAPI('/config/webhooks', data, 'POST', fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/webhooks/${id}`, data, 'PATCH', fetch);
}

function remove(id, fetch = undefined) {
  return postAPI(`/config/webhooks/${id}`, {}, 'DELETE', fetch, true);
}

export default {
  listUrl,
  addUrl,
  editUrl,

  successAddMsg,
  successUpdateMsg,
  successDeleteMsg,

  getList,
  get,
  add,
  update,
  remove
};
