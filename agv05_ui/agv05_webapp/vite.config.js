import { defineConfig } from 'vitest/config';
import { sveltekit } from '@sveltejs/kit/vite';
import { purgeCss } from 'vite-plugin-tailwind-purgecss';

export default defineConfig({
  plugins: [sveltekit(), purgeCss()],
  test: {
    include: ['src/**/*.{test,spec}.{js,ts}']
  },
  server: {
    port: 8080,
    host: '0.0.0.0',
    allowedHosts: true,
    proxy: {
      '/api/v3': {
        changeOrigin: false,
        target: 'http://localhost:8000'
      },
      '/logs': 'http://localhost:80',
      '/static': 'http://localhost:80',
      '/stream': 'http://localhost:80',
      '/admin': 'http://localhost:8000',
      '/system': 'http://localhost:8000',
      '/hashicon': 'http://localhost:8000',
      '/ws': {
        ws: true,
        target: 'http://localhost:9000',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/ws/, '')
      }
    }
  },
  define: {
    API_URL: '"/api/v3"',
    SYSTEM_URL: '"/system"',
    SOCK_URL: '"/ws/app"',
    SYSTEM_SOCK_URL: '"/ws/system"'
  },
  resolve: {
    alias: {
      components: '/src/components',
      stores: '/src/stores',
      actions: '/src/actions',
      'task-template-editor': '/src/routes/(config-panel)/config/task-templates/add',
      'map-layout-editor': '/src/routes/(config-panel)/config/maps/[id=integer]/layout/map',
      'mapx-layout-editor': '/src/routes/(config-panel)/config/maps/[id=integer]/layout/mapx',
      'map-annotation-editor':
        '/src/routes/(config-panel)/config/maps/[id=integer]/annotation/map_annotation',
      'mapx-annotation-editor':
        '/src/routes/(config-panel)/config/maps/[id=integer]/annotation/mapx_annotation',

      node_modules: '/node_modules',
      'cash-dom': '/src/lib/shared/cash-dom',
      d3: '/src/lib/shared/d3.js'
    }
  }
});
