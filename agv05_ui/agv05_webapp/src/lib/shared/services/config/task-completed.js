import { getAPI, downloadFileAPI } from '$lib/utils';

function listUrl(page = 1) {
  return `/config/task-completed${page === 1 ? '' : '?page=' + page}`;
}

function diagnosticUrl(id) {
  return `${API_URL}/config/task-completed/${id}/download-diagnostics`;
}

function getList(page = 0, limit = 100, fetch = undefined) {
  return getAPI(`/config/task-completed?limit=${limit}&offset=${(page - 1) * limit}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/task-completed/${id}`, fetch);
}

function getCompletedArchive(year = new Date().getFullYear(), fetch = undefined) {
  return getAPI(`/config/task-completed-archive?year=${year}`, fetch);
}

function downloadTaskCompleted(data) {
  return downloadFileAPI('/config/task-completed/download', data, 'POST');
}

function downloadDiagnostics(id) {
  return downloadFileAPI(`/config/task-completed/${id}/download-diagnostics`, {}, 'POST');
}

export default {
  listUrl,
  diagnosticUrl,

  get,
  getList,
  getCompletedArchive,
  downloadTaskCompleted,
  downloadDiagnostics
};
