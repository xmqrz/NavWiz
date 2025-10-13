import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import { purgeCss } from 'vite-plugin-tailwind-purgecss';

// https://vite.dev/config/
export default defineConfig({
  plugins: [svelte(), purgeCss()],
  base: './',
  root: './dashboard_src',
  build: {
    outDir: '../dashboard_build'
  },
  server: {
    port: 8082,
    host: '0.0.0.0',
    allowedHosts: true
  },
  define: {
    SOCK_URL: '"/ws/app"',
    SYSTEM_SOCK_URL: '"/ws/system"'
  },
  resolve: {
    alias: {
      hwapp_src: '/../hwapp_src',
      components: '/../src/components',
      stores: '/../src/stores',
      'user-panel': '../src/routes/(user-panel)',
      $lib: '/../src/lib',
      $app: '/../hwapp_src/components/app'
    }
  }
});
