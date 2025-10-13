import { getAPI, postAPI, downloadFileAPI } from '$lib/utils';

// REST API

function get() {
  return downloadFileAPI('/logs/custom', {}, 'GET');
}

function requestSystemLogs(data) {
  return postAPI('/config/log/request', data, 'POST');
}

function checkSystemLogs(data) {
  return postAPI('/config/log/download', data, 'PUT');
}

function downloadSystemLogs(data) {
  return downloadFileAPI('/config/log/download', data, 'POST');
}

function getMainLog(logPath = '', fetch = undefined) {
  return getAPI(`/config/log/main-logs/${logPath}`, fetch);
}

function getPeripheralLog(logPath = '', fetch = undefined) {
  return getAPI(`/config/log/peripheral-logs/${logPath}`, fetch);
}

export default {
  get,
  requestSystemLogs,
  checkSystemLogs,
  downloadSystemLogs,
  getMainLog,
  getPeripheralLog
};
