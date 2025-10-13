import { writable } from 'svelte/store';

// To create unique target for Select component
export const selectTargetCount = writable(0);
