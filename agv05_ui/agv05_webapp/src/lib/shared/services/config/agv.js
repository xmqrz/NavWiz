import { getAPI, postAPI } from '$lib/utils';

// MSGS
function successUpdateMsg(name) {
  // TODO: add translation here..
  return `AGV "${name}" updated.`;
}

function successUpdateDfleetModeMsg(url, isActive) {
  // TODO: add translation here..
  return `AGV settings updated and AGV is successfully paired with DFleet server.
    <br/>Current AGV status:
    <a href="${url}" target="_blank" class="anchor text-blue-200">
      ${isActive ? 'Activated' : 'Deactivated'}
    </a>`;
}

function successUpdateUUIDMsg() {
  // TODO: add translation here..
  return `AGV UUID updated.`;
}

// REST API
function get(fetch = undefined) {
  return getAPI(`/config/agv`, fetch);
}

function update(data, fetch = undefined) {
  return postAPI(`/config/agv`, data, 'POST', fetch);
}

function regenerateUUID(fetch = undefined) {
  return postAPI(`/config/agv/regenerate-uuid`, {}, 'POST', fetch);
}

export default {
  successUpdateMsg,
  successUpdateDfleetModeMsg,
  successUpdateUUIDMsg,

  get,
  update,
  regenerateUUID
};
