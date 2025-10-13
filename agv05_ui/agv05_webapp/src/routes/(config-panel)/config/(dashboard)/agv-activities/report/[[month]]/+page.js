import ac from '$lib/shared/services/config/agv-activities';
import stats from '$lib/shared/services/config/agv-statistics';
import { error } from '@sveltejs/kit';
import { parse } from 'date-fns';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  const configFetch = ac.getConfig(fetch);
  const dataFetch = stats.get(params.month || '', fetch);
  const [config, data] = await Promise.all([configFetch, dataFetch]);
  if (!config || !data || !data.current || !Array.isArray(data.results)) {
    throw error(404, 'Not Found');
  }

  const activityLabels = JSON.parse(config.value).labels;
  const activities = JSON.parse(config.activities);
  const acc = Object.assign(Object.fromEntries(activities), activityLabels);

  const current = pm(getUrlMonth(data.current));
  const currentYear = current.getFullYear();
  const currentMonth = current.getMonth() + 1;
  const next = data.next && getUrlMonth(data.next);
  const previous = data.previous && getUrlMonth(data.previous);
  const lastUpdate = new Date(data.last_update);

  const nDays = new Date(currentYear, currentMonth, 0).getDate();
  const days = [...Array(nDays).keys()];
  let aggregateDays = Math.floor((new Date() - current) / 86400 / 1000) + 1;
  aggregateDays = Math.max(Math.min(aggregateDays, nDays), 1);

  let byActivity = {};
  const total = {
    byActivity: {}
  };
  const extra = {
    mileage: [],
    mileageDelta: [],
    taskCounter: [],
    taskCounterDelta: []
  };
  let totalActivity = {};

  extra.mileage.length = nDays;
  extra.mileageDelta.length = nDays;
  extra.taskCounter.length = nDays;
  extra.taskCounterDelta.length = nDays;

  // collect and compute total
  let e0;
  if (data.result0) {
    e0 = JSON.parse(data.result0.by_activity).extra;
  }
  e0 = e0 || {};
  for (let result of data.results) {
    result.date = pd(result.date);
    result.by_activity = JSON.parse(result.by_activity);
    let day = result.date.getDate() - 1;

    let a = result.by_activity;
    let e = a.extra || {};
    e.mileage = e.mileage ?? e0.mileage;
    e.task_counter = e.task_counter ?? e0.task_counter;
    delete a.extra;

    for (let ac in a) {
      let duration = a[ac];
      let percentage = duration / 864.0;
      ac = acc[ac];

      if (!(ac in byActivity)) {
        byActivity[ac] = [];
        byActivity[ac][nDays] = acs();
        total.byActivity[ac] = acs();
      }
      if (!(day in byActivity[ac])) {
        byActivity[ac][day] = acs();
      }
      byActivity[ac][day].duration += duration;
      byActivity[ac][day].percentage += percentage;
      byActivity[ac][nDays].duration += duration;
      total.byActivity[ac].duration += duration;
    }

    extra.mileage[day] = e.mileage;
    extra.taskCounter[day] = e.task_counter;
    extra.mileageDelta[day] =
      e.mileage != null
        ? Math.max(0, e.mileage - (e0.mileage ?? 0))
        : e0.mileage != null
          ? 0
          : null;
    extra.taskCounterDelta[day] =
      e.task_counter != null
        ? Math.max(0, e.task_counter - (e0.task_counter ?? 0))
        : e0.task_counter != null
          ? 0
          : null;
    e0 = e;
  }

  // reconstruct byActivity.ac and total.byActivity.ac to list type.
  let seen = new Set();
  let acNames = [];
  for (let [k, _v] of activities) {
    let ac = acc[k];
    if (seen.has(ac)) {
      continue;
    }
    seen.add(ac);
    if (total.byActivity[ac]?.duration) {
      acNames.push(ac);
    }
  }
  total.byActivity = acNames.map((ac) => ({
    activity: ac,
    percentage: total.byActivity[ac].duration / 864.0 / aggregateDays,
    duration: total.byActivity[ac].duration
  }));
  for (let ac of acNames) {
    byActivity[ac][nDays].percentage = byActivity[ac][nDays].duration / 864.0 / aggregateDays;
  }
  byActivity = acNames.map((ac) => ({
    activity: ac,
    data: byActivity[ac]
  }));

  totalActivity = total.byActivity.reduce((acc, curr) => {
    const key = curr.activity;
    acc[key] = {
      percentage: curr.percentage,
      duration: curr.duration
    };
    return acc;
  }, {});

  let percentageMap = {};
  let durationMap = {};
  const activityList = [];

  for (let activityData of byActivity) {
    activityList.push(activityData.activity);
    if (!(activityData.activity in percentageMap)) {
      percentageMap[activityData.activity] = [];
      durationMap[activityData.activity] = [];
    }
    for (let data of activityData.data) {
      if (data) {
        percentageMap[activityData.activity].push(data.percentage);
        durationMap[activityData.activity].push(data.duration);
      } else {
        percentageMap[activityData.activity].push(0);
        durationMap[activityData.activity].push(0);
      }
    }
  }

  let dateList = ['date'];
  for (let i = 0; i < nDays; ++i) {
    dateList.push(currentYear + '-' + currentMonth + '-' + (i + 1));
  }

  let barChartPercentageData = [dateList];
  let barChartDurationData = [dateList];
  let barChartDailyTravelDistanceData = [dateList];
  let barChartDailyTaskCountData = [dateList];

  for (let [status, percentageArray] of Object.entries(percentageMap)) {
    barChartPercentageData.push([status, ...percentageArray]);
  }
  for (let [status, durationArray] of Object.entries(durationMap)) {
    barChartDurationData.push([status, ...durationArray]);
  }

  let dailyTravelDistanceList = [];
  let dailyTaskCountList = [];
  // Necessary to handle for current month
  for (let i = 0; i < nDays; ++i) {
    if (extra.mileageDelta[i]) {
      dailyTravelDistanceList.push(extra.mileageDelta[i]);
    } else {
      dailyTravelDistanceList.push(0);
    }
    if (extra.taskCounterDelta[i]) {
      dailyTaskCountList.push(extra.taskCounterDelta[i]);
    } else {
      dailyTaskCountList.push(0);
    }
  }
  barChartDailyTravelDistanceData.push([
    'Daily travel distance (m)',
    ...dailyTravelDistanceList
  ]);
  barChartDailyTaskCountData.push(['Daily task count', ...dailyTaskCountList]);

  return {
    current,
    next,
    previous,
    lastUpdate,
    days,
    byActivity,
    total,
    extra,
    totalActivity,
    barChartPercentageData,
    barChartDurationData,
    barChartDailyTravelDistanceData,
    barChartDailyTaskCountData,
    activityList
  };
};

function acs() {
  return { duration: 0, percentage: 0 };
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
