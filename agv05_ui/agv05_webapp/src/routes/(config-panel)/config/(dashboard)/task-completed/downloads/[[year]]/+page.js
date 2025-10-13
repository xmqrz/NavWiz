import stats from '$lib/shared/services/config/task-statistics';
import taskCompleted from '$lib/shared/services/config/task-completed';
import { error } from '@sveltejs/kit';
import { parse } from 'date-fns';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  const data = await stats.getTaskCompletedYearPagination(params.year || '', fetch);
  const monthlyDataSummary = await taskCompleted.getCompletedArchive(params.year, fetch);

  if (!data || !data.current || !Array.isArray(data.results)) {
    throw error(404, 'Not Found');
  }

  const current = py(getUrlYear(data.current));

  const next = data.next && getUrlYear(data.next);
  const previous = data.previous && getUrlYear(data.previous);

  return {
    current,
    next,
    previous,
    monthlyDataSummary
  };
};

function getUrlYear(url) {
  return new URL(url).searchParams.get('year');
}

// parse year
function py(yearStr) {
  return parse(yearStr, 'yyyy', new Date(2020, 0, 1));
}
