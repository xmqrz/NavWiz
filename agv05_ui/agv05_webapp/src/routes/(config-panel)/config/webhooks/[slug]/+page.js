import { error } from '@sveltejs/kit';
import { pagePermissions } from 'stores/auth.js';
import webhooks from '$lib/shared/services/config/webhooks';

/** @type {import('@sveltejs/kit').Load} */
export const load = async ({ fetch, params }) => {
  let add = params.slug === 'add';
  await pagePermissions([
    'system.view_system_panel',
    add ? 'system.add_webhook' : 'system.change_webhook'
  ]);
  if (add) {
    return {};
  }

  let data = await webhooks.get(params.slug, fetch);
  if (!data) {
    throw error(404, 'Not Found');
  }
  return {
    id: data.id,
    url: data.url,
    verify_ssl: data.verify_ssl,
    secret_token: data.secret_token,
    events: data.events
  };
};
