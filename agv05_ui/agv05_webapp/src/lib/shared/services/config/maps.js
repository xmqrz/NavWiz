import { getAPI, getAllAPI, postAPI } from '$lib/utils';

// URLS

function listUrl(page = 1) {
  return `/config/maps${page === 1 ? '' : '?page=' + page}`;
}

function addUrl() {
  return '/config/maps/add';
}

function editUrl(id) {
  return `/config/maps/${id}/edit`;
}

function copyUrl(id) {
  return `/config/maps/add?from=${id}`;
}

function layoutUrl(id) {
  return `/config/maps/${id}/layout`;
}

function annotationUrl(id) {
  return `/config/maps/${id}/annotation`;
}

function ocgsUrl(id, page = 1) {
  return `/config/maps/${id}/ocgs${page === 1 ? '' : '?page=' + page}`;
}

function ocgEditUrl(id, ocgId) {
  return `/config/maps/${id}/ocgs/${ocgId}/edit`;
}

function activeUrl() {
  return `/config/maps/active`;
}

function teleportUrl() {
  return `/config/maps/teleport`;
}

function paramUrl() {
  return `/config/maps/param`;
}

function tTriggerUrl() {
  return `/config/maps/transition-trigger`;
}

// MSGS

function successAddMsg(name) {
  // TODO: add translation here..
  return `Map "${name}" created.`;
}

function successUpdateMsg(name) {
  // TODO: add translation here..
  return `Map "${name}" updated.`;
}

function successUpdateOcgMsg(name) {
  // TODO: add translation here..
  return `Raw map "${name}" updated.`;
}

function successUpdateActiveMsg() {
  // TODO: add translation here..
  return 'Active map updated.';
}

function successUpdateTeleportMsg() {
  // TODO: add translation here..
  return 'Map teleportation updated.';
}

function successUpdateParamMsg() {
  // TODO: add translation here..
  return 'Map parameters updated.';
}

function successUpdateTTriggerMsg() {
  // TODO: add translation here..
  return 'Map transition trigger updated.';
}

function successDeleteMsg(name) {
  // TODO: add translation here..
  return `Map "${name}" deleted.`;
}

function successDeleteOcgMsg(name) {
  // TODO: add translation here..
  return `Raw map "${name}" deleted.`;
}

// REST API

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/maps?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function getAllList(cb = undefined, fetch = undefined) {
  return getAllAPI('/config/maps?limit=100', cb, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/maps/${id}`, fetch);
}

function add(id, data, fetch = undefined) {
  return postAPI(`/config/maps${id ? '?from=' + id : ''}`, data, 'POST', fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/maps/${id}`, data, 'PATCH', fetch);
}

function layout(id, fetch = undefined) {
  return getAPI(`/config/maps/${id}/layout`, fetch);
}

function annotation(id, fetch = undefined) {
  return getAPI(`/config/maps/${id}/annotation`, fetch);
}

function changesets(id, fetch = undefined) {
  return getAPI(`/config/maps/${id}/changesets?limit=100`, fetch);
}

function changeset(id, changesetId, fetch = undefined) {
  return getAPI(`/config/maps/${id}/changesets/${changesetId}`, fetch);
}

function getOcgList(id, page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/maps/${id}/ocgs?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function ocg(id, ocgId, fetch = undefined) {
  return getAPI(`/config/maps/${id}/ocgs/${ocgId}`, fetch);
}

function addOcg(id, data, fetch = undefined) {
  return postAPI(`/config/maps/${id}/ocgs`, data, 'POST', fetch);
}

function updateOcg(id, ocgId, data, fetch = undefined) {
  return postAPI(`/config/maps/${id}/ocgs/${ocgId}`, data, 'PUT', fetch);
}

function active(fetch = undefined) {
  return getAPI('/config/maps/active', fetch);
}

function teleport(fetch = undefined) {
  return getAPI('/config/maps/teleport', fetch);
}

function param(fetch = undefined) {
  return getAPI('/config/maps/param', fetch);
}

function tTrigger(fetch = undefined) {
  return getAPI('/config/maps/transition-trigger', fetch);
}

function updateLayout(id, data, fetch = undefined) {
  return postAPI(`/config/maps/${id}/layout`, data, 'POST', fetch);
}

function updateAnnotation(id, data, fetch = undefined) {
  return postAPI(`/config/maps/${id}/annotation`, data, 'POST', fetch);
}

function updateActive(data, fetch = undefined) {
  return postAPI(`/config/maps/active`, data, 'POST', fetch);
}

function updateTeleport(data, fetch = undefined) {
  return postAPI(`/config/maps/teleport`, data, 'POST', fetch);
}

function updateParam(data, fetch = undefined) {
  return postAPI(`/config/maps/param`, data, 'POST', fetch);
}

function updateTTrigger(data, fetch = undefined) {
  return postAPI(`/config/maps/transition-trigger`, data, 'POST', fetch);
}

function remove(id, fetch = undefined) {
  return postAPI(`/config/maps/${id}`, {}, 'DELETE', fetch, true);
}

function removeOcg(id, ocgId, fetch = undefined) {
  return postAPI(`/config/maps/${id}/ocgs/${ocgId}`, {}, 'DELETE', fetch, true);
}

export default {
  listUrl,
  addUrl,
  editUrl,
  copyUrl,
  layoutUrl,
  annotationUrl,
  ocgsUrl,
  ocgEditUrl,
  activeUrl,
  teleportUrl,
  paramUrl,
  tTriggerUrl,

  successAddMsg,
  successUpdateMsg,
  successUpdateOcgMsg,
  successUpdateActiveMsg,
  successUpdateTeleportMsg,
  successUpdateParamMsg,
  successUpdateTTriggerMsg,
  successDeleteMsg,
  successDeleteOcgMsg,

  getList,
  getAllList,
  get,
  add,
  update,
  layout,
  annotation,
  changesets,
  changeset,
  getOcgList,
  ocg,
  addOcg,
  updateOcg,
  active,
  teleport,
  param,
  tTrigger,
  updateLayout,
  updateAnnotation,
  updateActive,
  updateTeleport,
  updateParam,
  updateTTrigger,
  remove,
  removeOcg
};
