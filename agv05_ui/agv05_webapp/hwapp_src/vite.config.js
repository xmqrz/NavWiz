import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import { purgeCss } from 'vite-plugin-tailwind-purgecss';

// https://vite.dev/config/
export default defineConfig({
  plugins: [svelte(), purgeCss()],
  base: './',
  root: './hwapp_src',
  build: {
    outDir: '../hwapp_build'
  },
  server: {
    port: 8081,
    host: '0.0.0.0',
    allowedHosts: true
  },
  define: {
    SOCK_URL: '"/ws/app"',
    SYSTEM_SOCK_URL: '"/ws/app"'
  },
  resolve: {
    alias: {
      components: '/../src/components',
      stores: '/../src/stores',
      'user-panel': '../src/routes/(user-panel)',
      $lib: '/../src/lib',
      $app: '/components/app'
    }
  }
});
