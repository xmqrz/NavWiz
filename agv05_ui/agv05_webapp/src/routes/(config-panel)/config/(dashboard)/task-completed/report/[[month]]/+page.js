import stats from '$lib/shared/services/config/task-statistics';
import { error } from '@sveltejs/kit';
import { parse } from 'date-fns';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  const data = await stats.get(params.month || '', fetch);
  if (!data || !data.current || !Array.isArray(data.results)) {
    throw error(404, 'Not Found');
  }
  const statuses = ['Completed', 'Aborted', 'Cancelled'];

  const current = pm(getUrlMonth(data.current));
  const currentYear = current.getFullYear();
  const currentMonth = current.getMonth() + 1;
  const next = data.next && getUrlMonth(data.next);
  const previous = data.previous && getUrlMonth(data.previous);
  const lastUpdate = new Date(data.last_update);

  const nDays = new Date(currentYear, currentMonth, 0).getDate();
  const days = [...Array(nDays).keys()];

  const byStatus = {};
  const byTt = {};
  const total = {
    byStatus: {},
    byTt: {}
  };
  const totalTt = {};
  let statusList = [];

  for (let st of statuses) {
    statusList.push(st);
    byStatus[st] = [];
    byStatus[st][nDays] = ts();
    byTt[st] = {};
    total.byStatus[st] = ts();
    total.byTt[st] = {};
  }

  // collect and compute total
  for (let result of data.results) {
    result.date = pd(result.date);
    result.by_status = JSON.parse(result.by_status);
    result.by_tt = JSON.parse(result.by_tt);
    let day = result.date.getDate() - 1;

    for (let st of statuses) {
      let s = result.by_status[st];
      byStatus[st][day] = s;
      tsAdd(byStatus[st][nDays], s);
      tsAdd(total.byStatus[st], s);

      for (let tt of result.by_tt[st]) {
        if (!(tt.task_template in byTt[st])) {
          byTt[st][tt.task_template] = [];
          byTt[st][tt.task_template][nDays] = ts();
          total.byTt[st][tt.task_template] = ts();
        }
        byTt[st][tt.task_template][day] = tt;
        tsAdd(byTt[st][tt.task_template][nDays], tt);
        tsAdd(total.byTt[st][tt.task_template], tt);
      }
    }
  }

  // reconstruct 'byTt.Status.tt' and 'total.byTt.Status.tt' to list type.
  for (let st of statuses) {
    let tbtt = total.byTt[st];
    let ttNames = Object.keys(tbtt).sort();

    total.byTt[st] = ttNames.map((tt) => ({
      taskTemplate: tt,
      count: tbtt[tt]['count'],
      duration: tbtt[tt]['duration']
    }));

    let btt = byTt[st];
    byTt[st] = ttNames.map((tt) => ({
      taskTemplate: tt,
      data: btt[tt]
    }));
  }

  for (let st of statuses) {
    for (let tt of total.byTt[st]) {
      if (!(tt.taskTemplate in totalTt)) {
        totalTt[tt.taskTemplate] = ts();
      }
      tsAdd(totalTt[tt.taskTemplate], tt);
    }
  }

  let countMap = {};
  let durationMap = {};

  for (let st of statusList) {
    countMap[st] = [];
    durationMap[st] = [];
    for (let data of byStatus[st]) {
      if (data) {
        countMap[st].push(data.count);
        durationMap[st].push(data.duration);
      } else {
        countMap[st].push(0);
        durationMap[st].push(0);
      }
    }
  }

  let dateList = ['date'];
  for (let i = 0; i < nDays; ++i) {
    dateList.push(currentYear + '-' + currentMonth + '-' + (i + 1));
  }

  let barChartCountData = [dateList];
  let barChartDurationData = [dateList];

  for (let [status, countArray] of Object.entries(countMap)) {
    barChartCountData.push([status, ...countArray]);
  }
  for (let [status, durationArray] of Object.entries(durationMap)) {
    barChartDurationData.push([status, ...durationArray]);
  }

  return {
    statuses,
    current,
    next,
    previous,
    lastUpdate,
    days,
    byStatus,
    byTt,
    total,
    totalTt,
    barChartCountData,
    barChartDurationData,
    statusList
  };
};

function ts() {
  return { count: 0, duration: 0 };
}

function tsAdd(ts, ts2) {
  ts.count += ts2.count;
  ts.duration += ts2.duration;
}

function getUrlMonth(url) {
  return new URL(url).searchParams.get('month');
}

// parse month
function pm(monthStr) {
  return parse(monthStr, 'yyyy-MM', new Date(2020, 0, 1));
}

// parse date
function pd(dateStr) {
  return parse(dateStr, 'yyyy-MM-dd', new Date(2020, 0, 1));
}
