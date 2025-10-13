import { postAPI } from '$lib/utils';
import { getAPI } from '$lib/utils';

export const agv05Task = {
  queryCompleted: function (limit = 50) {
    return getAPI(`/tasks/completed?limit=${limit}`);
  },
  templates: function () {
    return getAPI(`/tasks/templates`);
  },
  post: function (data) {
    return postAPI('/tasks', data);
  },
  abort: function (id) {
    return postAPI(`/tasks/${id}/abort`, {});
  },
  cancel: function (id, fmsTaskId) {
    return postAPI(`/tasks/${id || 0}/cancel`, {
      fms_task_id: fmsTaskId
    });
  },
  prioritize: function (id, fmsTaskId) {
    return postAPI(`/tasks/${id || 0}/prioritize`, {
      fms_task_id: fmsTaskId
    });
  },
  suspend: function (id, fmsTaskId) {
    return postAPI(`/tasks/${id || 0}/suspend`, {
      fms_task_id: fmsTaskId
    });
  },
  resume: function (id, fmsTaskId) {
    return postAPI(`/tasks/${id || 0}/resume`, {
      fms_task_id: fmsTaskId
    });
  }
};
