import forms from '@tailwindcss/forms';
import { join } from 'path';
import { skeleton } from '@skeletonlabs/tw-plugin';
import { navwizTheme } from './src/navwiz-theme.js';

/** @type {import('tailwindcss').Config} */
export default {
  darkMode: 'class',
  content: [
    './src/**/*.{html,js,svelte,ts}',
    join(require.resolve('@skeletonlabs/skeleton'), '../**/*.{html,js,svelte,ts}')
  ],
  theme: {
    extend: {
      aspectRatio: {
        '4/3': '4 / 3'
      }
    }
  },
  plugins: [
    forms,
    skeleton({
      themes: {
        custom: [navwizTheme]
      }
    })
  ]
};
