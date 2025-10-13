import { postAPI } from '$lib/utils';

export const agv05Transaction = {
  abort: function (id) {
    return postAPI(`/transactions/${id}/abort`, {});
  },
  cancel: function (id) {
    return postAPI(`/transactions/${id}/cancel`, {});
  },
  resume: function (id, action, error) {
    return postAPI(`/transactions/${id}/resume`, {
      action: action,
      error: error
    });
  }
};
