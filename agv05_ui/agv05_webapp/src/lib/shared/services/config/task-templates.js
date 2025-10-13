import { getAPI, postAPI } from '$lib/utils';

// URLS

function listUrl(page = 1, category = undefined, isActive = undefined, isTopLvl = undefined) {
  let query = [];
  if (page !== 1) {
    query.push(`page=${page}`);
  }
  if (category) {
    query.push(`category=${category}`);
  }
  if (isActive) {
    query.push(`is_active=${isActive}`);
  }
  if (isTopLvl) {
    query.push(`is_top_level=${isTopLvl}`);
  }
  return `/config/task-templates${query.length ? '?' + query.join('&') : ''}`;
}

function addUrl() {
  return '/config/task-templates/add';
}

function editUrl(id) {
  return `/config/task-templates/${id}/edit`;
}

function copyUrl(id) {
  return `/config/task-templates/add?from=${id}`;
}

function usersUrl(id) {
  return `/config/task-templates/${id}/users`;
}

function viewSavedUrl(id) {
  return `/config/task-templates/${id}/last-saved`;
}

function viewCachedUrl(id) {
  return `/config/task-templates/${id}/last-cached`;
}

function globalParamUrl() {
  return `/config/task-templates/global-param`;
}

function variableUrl() {
  return `/config/task-templates/variable`;
}

function massUpdateUrl(ttList) {
  return `/config/task-templates/mass-update?tt=${ttList.join(',')}`;
}

function massCategoryUpdateUrl(ttList) {
  return `/config/task-templates/mass-category-update?tt=${ttList.join(',')}`;
}

function massUsersUpdateUrl(ttList) {
  return `/config/task-templates/mass-users?tt=${ttList.join(',')}`;
}

function massDeleteUrl(ttList) {
  return `/config/task-templates/mass-delete?tt=${ttList.join(',')}`;
}

// MSGS

function successAddMsg(name) {
  // TODO: add translation here..
  return `Task template "${name}" created.`;
}

function successUpdateMsg(name) {
  // TODO: add translation here..
  return `Task template "${name}" updated.`;
}

function successUpdateGlobalParam() {
  // TODO: add translation here..
  return 'Task template global parameters updated.';
}

function successUpdateVariable() {
  // TODO: add translation here..
  return 'Task template variables updated.';
}

function successDeleteMsg(name) {
  // TODO: add translation here..
  return `Task template "${name}" deleted.`;
}

function successMassUpdateMsg(num) {
  // TODO: add translation here..
  return `${num} task templates updated.`;
}

function successMassDeleteMsg(num) {
  // TODO: add translation here..
  return `${num} task templates deleted.`;
}

// REST API

function meta(fetch = undefined) {
  return getAPI('/config/task-templates/meta', fetch);
}

function getList(
  page = 1,
  limit = 100,
  category = undefined,
  isActive = undefined,
  isTopLvl = undefined,
  fetch = undefined
) {
  let query = [`limit=${limit}`];
  if (page > 1) {
    query.push(`offset=${(page - 1) * limit}`);
  }
  if (category) {
    query.push(`category=${category}`);
  }
  if (isActive) {
    query.push(`is_active=${isActive}`);
  }
  if (isTopLvl) {
    query.push(`is_top_level=${isTopLvl}`);
  }
  return getAPI(`/config/task-templates?${query.join('&')}`, fetch);
}

function get(id, fetch = undefined) {
  return getAPI(`/config/task-templates/${id}`, fetch);
}

function getCached(id, fetch = undefined) {
  return getAPI(`/config/task-templates/${id}/last-cached`, fetch);
}

function getUsers(id, fetch = undefined) {
  return getAPI(`/config/task-templates/${id}/users`, fetch);
}

function getGlobalParam(fetch = undefined) {
  return getAPI(`/config/task-templates/global-param`, fetch);
}

function getVariable(fetch = undefined) {
  return getAPI(`/config/task-templates/variable`, fetch);
}

function add(data, fetch = undefined) {
  return postAPI(`/config/task-templates`, data, 'POST', fetch);
}

function update(id, data, fetch = undefined) {
  return postAPI(`/config/task-templates/${id}`, data, 'PATCH', fetch);
}

function updateUsers(id, data, fetch = undefined) {
  return postAPI(`/config/task-templates/${id}/users`, data, 'POST', fetch);
}

function updateGlobalParam(data, fetch = undefined) {
  return postAPI(`/config/task-templates/global-param`, data, 'POST', fetch);
}

function updateVariable(data, fetch = undefined) {
  return postAPI(`/config/task-templates/variable`, data, 'POST', fetch);
}

function deleteTaskTemplate(id, fetch = undefined) {
  return postAPI(`/config/task-templates/${id}`, {}, 'DELETE', fetch, true);
}

function getMassUpdate(ttParam, fetch = undefined) {
  return getAPI(`/config/task-templates/mass-update?tt=${ttParam}`, fetch);
}

function massUpdate(ttList, data, fetch = undefined) {
  return postAPI(
    `/config/task-templates/mass-update?tt=${ttList.join(',')}`,
    data,
    'POST',
    fetch
  );
}

function getMassCategoryUpdate(ttParam, fetch = undefined) {
  return getAPI(`/config/task-templates/mass-category-update?tt=${ttParam}`, fetch);
}

function massCategoryUpdate(ttList, data, fetch = undefined) {
  return postAPI(
    `/config/task-templates/mass-category-update?tt=${ttList.join(',')}`,
    data,
    'POST',
    fetch
  );
}

function getMassUsersUpdate(ttParam, fetch = undefined) {
  return getAPI(`/config/task-templates/mass-users?tt=${ttParam}`, fetch);
}

function massUsersUpdate(ttList, data, fetch = undefined) {
  return postAPI(
    `/config/task-templates/mass-users?tt=${ttList.join(',')}`,
    data,
    'POST',
    fetch
  );
}

function getMassDelete(ttParam, fetch = undefined) {
  return getAPI(`/config/task-templates/mass-delete?tt=${ttParam}`, fetch);
}

function massDelete(ttList, fetch = undefined) {
  return postAPI(
    `/config/task-templates/mass-delete?tt=${ttList.join(',')}`,
    {},
    'POST',
    fetch
  );
}

export default {
  listUrl,
  addUrl,
  editUrl,
  copyUrl,
  usersUrl,
  viewSavedUrl,
  viewCachedUrl,
  globalParamUrl,
  variableUrl,
  massUpdateUrl,
  massCategoryUpdateUrl,
  massUsersUpdateUrl,
  massDeleteUrl,

  successAddMsg,
  successUpdateMsg,
  successUpdateGlobalParam,
  successUpdateVariable,
  successDeleteMsg,
  successMassUpdateMsg,
  successMassDeleteMsg,

  meta,
  add,
  getList,
  get,
  getCached,
  getUsers,
  getGlobalParam,
  getVariable,
  update,
  updateUsers,
  updateGlobalParam,
  updateVariable,
  delete: deleteTaskTemplate,
  getMassUpdate,
  massUpdate,
  getMassCategoryUpdate,
  massCategoryUpdate,
  getMassUsersUpdate,
  massUsersUpdate,
  getMassDelete,
  massDelete
};
