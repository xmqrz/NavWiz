import { getAPI, postAPI } from '$lib/utils';

function url() {
  return '/config/maintenance';
}

function pmUrl() {
  return '/config/maintenance/preventive-maintenance';
}

function serviceLogsUrl() {
  return '/config/maintenance/service-logs';
}

function linkUrl() {
  return `${API_URL}/config/assembly-info/link`;
}

function successUpdateMsg() {
  return 'AMR info updated.';
}

function successUploadMsg() {
  return 'AMR and parts info updated.';
}

function successSyncMsg() {
  return 'AMR and parts info populated from DF Hub.';
}

function pmSuccessUpdateMsg() {
  return 'Preventive maintenance data updated.';
}

function slogSuccessUploadMsg() {
  return 'Service logs updated.';
}

function slogSuccessSyncMsg() {
  return 'Service logs populated from DF Hub.';
}

function get(fetch = undefined) {
  return getAPI('/config/assembly-info', fetch);
}

function update(data, fetch = undefined) {
  return postAPI('/config/assembly-info', data, 'POST', fetch);
}

function sync(fetch = undefined) {
  return postAPI('/config/assembly-info/sync', {}, 'POST', fetch);
}

function getPm(fetch = undefined) {
  return getAPI('/config/preventive-maintenance', fetch);
}

function updatePm(data, fetch = undefined) {
  return postAPI('/config/preventive-maintenance', data, 'POST', fetch);
}

function getServiceLogs(data, fetch = undefined) {
  return getAPI('/config/service-logs', fetch);
}

function updateServiceLogs(data, fetch = undefined) {
  return postAPI('/config/service-logs', data, 'POST', fetch);
}

function syncServiceLogs(fetch = undefined) {
  return postAPI('/config/service-logs/sync', {}, 'POST', fetch);
}

export default {
  url,
  pmUrl,
  serviceLogsUrl,
  linkUrl,

  successUpdateMsg,
  successUploadMsg,
  successSyncMsg,
  pmSuccessUpdateMsg,
  slogSuccessUploadMsg,
  slogSuccessSyncMsg,

  get,
  update,
  sync,
  getPm,
  updatePm,
  getServiceLogs,
  updateServiceLogs,
  syncServiceLogs
};
