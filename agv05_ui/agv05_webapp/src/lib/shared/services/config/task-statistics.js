import { getAPI } from '$lib/utils';

function get(month, fetch = undefined) {
  return getAPI(`/task-statistics?month=${month}`, fetch);
}

function getTaskCompletedYearPagination(year, fetch = undefined) {
  return getAPI(`/task-completed-year-pagination?year=${year}`, fetch);
}

export default {
  get,
  getTaskCompletedYearPagination
};
