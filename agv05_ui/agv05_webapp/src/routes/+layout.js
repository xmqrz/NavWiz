import { loadTranslations } from '$lib/translations';
import { initAuth } from 'stores/auth.js';
import { initClock } from 'stores/server-clock.js';
import logo from '$lib/assets/img/navwiz_logo.svg';

export const ssr = false;
export const trailingSlash = 'never';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch }) => {
  const loadingPanel = document.querySelector('.navwiz-loading-panel');
  initLoading(loadingPanel);

  await Promise.all([initAuth(fetch), initClock(fetch)]);

  clearLoading(loadingPanel);

  const initialLocale = 'en'; // get from cookie / url / fetch from server...
  await loadTranslations(initialLocale); // keep this just before the `return`
  return {};
};

// TODO: handle in svelte.
let loadingTimeout;
function initLoading(loadingPanel) {
  if (!loadingPanel) {
    return;
  }
  loadingTimeout = setTimeout(() => {
    loadingPanel.style.display = 'flex';
    const img = loadingPanel.querySelector('img.logo');
    if (img) {
      img.src = logo;
    }
  }, 3000);
}

function clearLoading(loadingPanel) {
  if (loadingPanel) {
    loadingPanel.remove();
  }
  if (loadingTimeout) {
    clearTimeout(loadingTimeout);
  }
}
