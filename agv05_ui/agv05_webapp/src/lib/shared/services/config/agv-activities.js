import { getAPI, postAPI, downloadFileAPI } from '$lib/utils';

function listUrl(page = 1) {
  return `/config/agv-activities${page === 1 ? '' : '?page=' + page}`;
}

function successUpdateActivityMsg() {
  return 'AGV activity updated.';
}

function successUpdateConfigMsg() {
  return 'AGV activity configuration updated.';
}

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/agv-activities?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/agv-activities/${id}`, fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/agv-activities/${id}`, data, 'PATCH', fetch);
}

function getConfig(fetch = undefined) {
  return getAPI('/config/agv-activities/configuration', fetch);
}

function updateConfig(data, fetch = undefined) {
  return postAPI('/config/agv-activities/configuration', data, 'POST', fetch);
}

function getActivityChoices(fetch = undefined) {
  return getAPI('/config/agv-activities/get-activity-choices', fetch);
}

function getCompletedArchive(year = new Date().getFullYear(), fetch = undefined) {
  return getAPI(`/config/agv-activities-archive?year=${year}`, fetch);
}

function downloadAgvActivities(data) {
  return downloadFileAPI('/config/agv-activities/download', data, 'POST');
}

export default {
  listUrl,

  successUpdateActivityMsg,
  successUpdateConfigMsg,

  get,
  update,
  getList,
  getConfig,
  updateConfig,
  getActivityChoices,
  getCompletedArchive,
  downloadAgvActivities
};
