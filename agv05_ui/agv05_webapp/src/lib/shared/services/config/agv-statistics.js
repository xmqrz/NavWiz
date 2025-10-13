import { getAPI } from '$lib/utils';

function get(month, fetch = undefined) {
  return getAPI(`/agv-statistics?month=${month}`, fetch);
}

function getAgvActivitiesYearPagination(year, fetch = undefined) {
  return getAPI(`/agv-activities-year-pagination?year=${year}`, fetch);
}

export default {
  get,
  getAgvActivitiesYearPagination
};
