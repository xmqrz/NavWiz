import {
  parseISO,
  differenceInMilliseconds,
  addMilliseconds,
  isSameDay,
  isValid
} from 'date-fns';
import { formatInTimeZone, getTimezoneOffset } from 'date-fns-tz';
import { readable, derived } from 'svelte/store';
import { get } from 'svelte/store';

let offset = 0;
let tz = 'Asia/Kuala_Lumpur';

export async function initClock(fetch) {
  try {
    const response = await fetch(API_URL + '/time');
    if (response.ok) {
      const now = new Date();

      const result = await response.json();
      const serverNow = parseISO(result.now);
      tz = result.tz;
      if (!isNaN(serverNow)) {
        offset = differenceInMilliseconds(serverNow, now);
      }
    } else {
      throw new Error(response.statusText);
    }
  } catch (error) {
    console.error('Fail to get backend time:', error);
  }
}

export async function syncClock(result) {
  const now = new Date();
  const serverNow = parseISO(result.now);
  if (!isNaN(serverNow)) {
    offset = differenceInMilliseconds(serverNow, now);
  }
}

export const serverClock = readable([new Date(), tz], (set) => {
  function update() {
    const serverNow = addMilliseconds(new Date(), offset);
    set([serverNow, tz]);
  }
  const timerId = setInterval(update, 1000);
  update();

  return () => {
    clearInterval(timerId);
  };
});

export const serverClockFormat = (format = 'MMM d, yyyy, h:mm:ss a') => {
  return derived(serverClock, ($serverClock) => {
    return formatInTimeZone($serverClock[0], $serverClock[1], format);
  });
};

export function timeFormat(time, format = 'yyyy-MM-dd HH:mm:ss.SSS') {
  // format in server timezone
  if (!isValid(time)) {
    return;
  }
  return formatInTimeZone(time, tz, format);
}

export function unixFormat(unixtime, format) {
  const time = new Date(unixtime * 1000);
  return timeFormat(time, format);
}

// use for table display
export function timeDisplay(isotimeStr) {
  if (!isotimeStr) {
    return '-';
  }
  let time = parseISO(isotimeStr);
  if (isSameDay(time, get(serverClock)[0])) {
    return timeFormat(time, 'h:mm:ss aaaa') || '-';
  }
  return timeFormat(time, 'h:mm:ss aaaa dd/MM/yyyy') || '-';
}

export function monoTimeDisplay(isotimeStr) {
  if (!isotimeStr) {
    return '-';
  }
  let time = parseISO(isotimeStr);
  if (isSameDay(time, get(serverClock)[0])) {
    return timeFormat(time, 'HH:mm') || '-';
  }
  return timeFormat(time, 'dd/MM/yyyy HH:mm') || '-';
}

export function toServerTZ(date, utc = false) {
  const o = new Date().getTimezoneOffset() * 60 * 1000;
  const so = -getTimezoneOffset(tz); // -ve to make output similar to date.getTimezoneOffset
  const utcDate = utc ? date : addMilliseconds(date, o);
  return addMilliseconds(utcDate, -so);
}
