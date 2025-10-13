import { validationDataChannel } from 'stores/sock/index.js';
import { readable } from 'svelte/store';

export const validationData = readable({}, (set) => {
  validationDataChannel.subscribe(set);
  return () => {
    validationDataChannel.unsubscribe(set);
  };
});
