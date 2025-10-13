import { getAPI, postAPI } from '$lib/utils';

function successUpdateMsg() {
  return 'License key updated.';
}

function successReturnMsg() {
  return 'License key returned.';
}

function unilateralReturnMsg() {
  return 'License key returned unilaterally.';
}

function info(fetch = undefined) {
  return getAPI('/config/license/info', fetch);
}

function req(fetch = undefined) {
  return getAPI('/config/license', fetch);
}

function offline(data, fetch = undefined) {
  return postAPI('/config/license', data, 'POST', fetch);
}

function online(data, fetch = undefined) {
  return postAPI('/config/license/online', data, 'POST', fetch);
}

function ret(fetch = undefined) {
  return postAPI('/config/license/return', {}, 'POST', fetch);
}

export default {
  successUpdateMsg,
  successReturnMsg,
  unilateralReturnMsg,
  info,
  req,
  online,
  offline,
  ret
};
