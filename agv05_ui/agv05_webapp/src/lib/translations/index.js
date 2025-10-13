// https://github.com/sveltekit-i18n/lib/blob/master/examples/one-page/src/lib/translations/index.js
import i18n from 'sveltekit-i18n';
import lang from './lang.json';

/** @type {import('sveltekit-i18n').Config} */
export const config = {
  translations: {
    en: { lang }
  },
  loaders: [
    {
      locale: 'en',
      key: 'user_panel',
      loader: async () => (await import('./en/user_panel.json')).default
    },
    {
      locale: 'en',
      key: 'config_panel',
      loader: async () => (await import('./en/config_panel.json')).default
    },
    {
      locale: 'en',
      key: 'common',
      loader: async () => (await import('./en/common.json')).default
    }
  ]
};

export const { t, loading, locales, locale, loadTranslations } = new i18n(config);

loading.subscribe(($loading) => $loading);
