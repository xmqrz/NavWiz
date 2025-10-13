import { getAPI, formAPI } from '$lib/utils';

function successUpdateMsg() {
  return 'Patch is now in progress. Please do not power off the PC during the update.';
}

function get(fetch = undefined) {
  return getAPI('/config/software-patch', fetch);
}

function update(data, fetch = undefined) {
  return formAPI('/config/software-patch', data, 'POST', fetch);
}

export default {
  successUpdateMsg,
  get,
  update
};
