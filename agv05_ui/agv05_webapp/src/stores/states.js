import { writable } from 'svelte/store';

export const robotRunning = writable(-1);
export const robotName = writable('');
export const countdownTimer = writable(0);
export const popupType = writable('');
export const sockjsConnected = writable(false);
