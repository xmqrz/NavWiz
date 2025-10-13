import { format } from 'date-fns';

export function csvReport(data, tab) {
  let result = `Completed Tasks Report (${capitalize(tab)}) for ${dm(data.current)}\n`;

  let days = csvDays(data.days);

  result += 'Status' + days;
  for (let st of data.statuses) {
    result += st + csvRow(data.byStatus[st], tab);
  }

  for (let st of data.statuses) {
    result += `\nBreakdown: ${st} Tasks\n`;
    result += 'Task Template' + days;

    for (let tt of data.byTt[st]) {
      result += tt.taskTemplate + csvRow(tt.data, tab);
    }
  }
  return result;
}

function csvDays(days) {
  return `,${days.map((v) => v + 1).join()},Total\n`;
}

function csvRow(dataRow, tab) {
  let result = `,${[...dataRow].map(tab !== 'duration' ? dc : dd).join()}\n`;
  if (tab === 'combined') {
    result += `,${[...dataRow].map(dd).join()}\n`;
  }
  return result;
}

function capitalize(s) {
  return s[0].toUpperCase() + s.slice(1);
}

// display month
function dm(month) {
  return format(month, 'MMMM yyyy');
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

// display count
function dc(res) {
  return res?.count || '-';
}
