import adapter from '@sveltejs/adapter-static';
import { vitePreprocess } from '@sveltejs/kit/vite';

// HACK: prevent removing folder for overlay
function customAdapter(options) {
  var data = adapter(options);
  var origAdapt = data.adapt;
  data.adapt = async function (builder) {
    var origRimraf = builder.rimraf;
    builder.rimraf = function (path) {
      if (path === 'build') {
        path = 'build/*';
      }
      return origRimraf(path);
    };
    return origAdapt(builder);
  };
  return data;
}

/** @type {import('@sveltejs/kit').Config} */
const config = {
  vitePlugin: {
    inspector: {
      toggleKeyCombo: 'alt-x'
    }
  },
  preprocess: vitePreprocess(),
  kit: {
    adapter: customAdapter({
      pages: 'build',
      assets: 'build',
      fallback: 'index.html',
      precompress: false,
      strict: true
    }),
    prerender: {
      handleHttpError: ({ path, referrer, message }) => {
        // ignore deliberate link to shiny 404 page
        if (path === '/not-found' && referrer === '/blog/how-we-built-our-404-page') {
          return;
        }

        // otherwise fail the build
        throw new Error(message);
      }
    }
  }
};

export default config;
