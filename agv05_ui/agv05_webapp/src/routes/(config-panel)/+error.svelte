<script>
  import * as _ from 'lodash-es';
  import { afterNavigate } from '$app/navigation';
  import { page } from '$app/stores';

  import { whiteLabel } from 'stores/white-label.js';

  let title;

  afterNavigate(() => {
    if ($page.route.id.startsWith('/(config-panel)/config/')) {
      const regex = /\/config\/([^/]+)/;
      const match = $page.route.id.match(regex);
      if (match) {
        title = _.startCase(match[1]);
      }
    }
  });

  const Status = {
    FmsVoid: 406,
    AccessDenied: 403
  };
</script>

<div class="p-5">
  <div class="flex space-x-5 py-5">
    <h1 class="text-3xl tracking-widest">{$page.error.title || title || 'Error'}</h1>
  </div>
  <hr />
</div>
<div class="p-9">
  {#if $page.status === Status.AccessDenied}
    {#if $page.error.licenseVoid}
      <div>
        {$page.error.data} Please enter a new license key
        <a href="/config/license" class="anchor"><strong>here</strong></a>.
      </div>
    {:else}
      <div>
        {$page.error.data}
      </div>
    {/if}
  {:else if $page.status === Status.FmsVoid}
    <div>
      This setting is not available when activating DFleet mode. Please edit it in the
      <a href={$page.error.data} class="anchor" target="_blank">
        <strong>DFleet panel</strong>
      </a>.
    </div>
  {:else}
    <div class="flex h-full flex-col items-center justify-center p-5 text-2xl">
      <div>{$page.status}</div>
      <div>{$page.error.message}</div>
    </div>
  {/if}
</div>
<div class="p-5 pt-20">
  <hr />
  <!-- eslint-disable-next-line svelte/no-at-html-tags -->
  <p class="pt-5">NavWiz ConfigPanel. {@html $whiteLabel.copyright_label || ''}</p>
</div>
