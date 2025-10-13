import { readable } from 'svelte/store';

export const whiteLabel = readable({}, (set) => {
  fetch(API_URL + '/white-label')
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
      console.error('Fail to get backend white label:', error);
    });

  return () => {};
});
