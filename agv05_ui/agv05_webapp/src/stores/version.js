import { readable } from 'svelte/store';

export const version = readable({}, (set) => {
  fetch(API_URL + '/version')
    .then((response) => {
      if (response.ok) {
        return response.json();
      } else {
        throw new Error(response.statusText);
      }
    })
    .then((result) => {
      set(result);
    })
    .catch((error) => {
      console.error('Fail to get backend version:', error);
    });

  return () => {};
});
