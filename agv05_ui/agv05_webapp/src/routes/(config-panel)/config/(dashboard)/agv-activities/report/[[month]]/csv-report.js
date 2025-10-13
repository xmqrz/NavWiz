import { format } from 'date-fns';

export function csvReport(data, tab) {
  let result = `AGV Activities Report for ${dm(data.current)}\n`;

  let days = csvDays(data.days);

  if (['percentage', 'duration'].indexOf(tab) >= 0) {
    result += `Activity${days},Total\n`;
    for (let ac of data.byActivity) {
      result += `${ac.activity},${[...ac.data].map(tab === 'percentage' ? dp : dd).join()}\n`;
    }
  } else {
    result += `Mileage${days}\n`;
    result += `Absolute Mileage (m),${[...data.extra.mileage].map(de).join()}\n`;
    result += `Daily Travel Distance (m),${[...data.extra.mileageDelta].map(de).join()}\n`;
    result += '\n';
    result += `Task Counter${days}\n`;
    result += `Absolute Task Counter,${[...data.extra.taskCounter].map(dt).join()}\n`;
    result += `Daily Task Count,${[...data.extra.taskCounterDelta].map(dt).join()}\n`;
  }
  return result;
}

function csvDays(days) {
  return `,${days.map((v) => v + 1).join()}`;
}

// display month
function dm(month) {
  return format(month, 'MMMM yyyy');
}

// display percentage
function dp(res) {
  let percentage = res?.percentage;
  if (!percentage) {
    return '-';
  }
  return percentage.toFixed(2) + '%';
}

// display duration
function dd(res) {
  let seconds = res?.duration;
  if (!seconds) {
    return '-';
  }
  let hours = Math.floor(seconds / 3600);
  let minutes = Math.floor((seconds % 3600) / 60);
  seconds %= 60;
  return `${hours}:${('0' + minutes).slice(-2)}:${('0' + seconds).slice(-2)}`;
}

// display mileage
function de(mileage) {
  if (mileage == null) {
    return '-';
  }
  return mileage.toFixed(1);
}

// display task counter
function dt(taskCounter) {
  return taskCounter ?? '-';
}
