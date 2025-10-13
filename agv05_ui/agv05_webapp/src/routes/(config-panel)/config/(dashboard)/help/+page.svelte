<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Perm from 'components/Perm.svelte';
  import { whiteLabel } from 'stores/white-label.js';

  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount, onDestroy } from 'svelte';
  import { alertModal } from '$lib/modal-service.js';
  import { remoteAssistChannel } from 'stores/sock';

  export let data;

  var licenseStatus = getLicenseStatus(data.licenseInfo);

  function getLicenseStatus(r) {
    if (r.valid) {
      if (!(r.features & 1)) {
        return `This product has been licensed to <i>${r.owner} (${r.email})</i> for trial purpose only. You have ${r.days} day(s) remaining to evaluate this software.`;
      } else if (r.days) {
        return `This product has been licensed to <i>${r.owner} (${r.email})</i> under subscription basis. Subscription will be renewed on ${new Date(r.till * 1000).toLocaleString()}.`;
      } else {
        return `This product has been licensed to <i>${r.owner} (${r.email})</i>.`;
      }
    } else {
      if (!r.features) {
        return 'The license key is invalid.';
      } else if (r.days < 0) {
        if (r.features & 1) {
          return 'Your subscription license has expired. Please renew your license.';
        } else {
          return 'Your trial license has expired. Please consider purchasing a new license.';
        }
      } else {
        return 'Some features are not licensed.';
      }
    }
  }

  let remoteConnected = false;
  const modalStore = getModalStore();

  function remoteConnect() {
    remoteAssistChannel.publish({ id: 'remote_connect' });
  }
  function remoteDisconnect() {
    remoteAssistChannel.publish({ id: 'remote_disconnect' });
  }
  function doRefresh() {
    window.location.reload();
  }
  function remoteAssistCallback(data) {
    if (data.id === 'status') {
      remoteConnected = data.value;
    } else if (data.id === 'error') {
      modalStore.trigger(alertModal('Failed to start remote assistance.'));
    }
  }
  onMount(() => {
    remoteAssistChannel.subscribe(remoteAssistCallback);
  });
  onDestroy(() => {
    remoteAssistChannel.unsubscribe(remoteAssistCallback);
  });
</script>

<ConfigLayout title="Help Page" validation={false}>
  <div class="flex max-w-3xl flex-col gap-7">
    <div class="card h-full w-full shadow-lg">
      <header class="card-header p-3 px-7 font-semibold">About NavWiz</header>
      <hr />
      <section class="space-y-3 p-7">
        <h3 class="text-3xl">{data.licenseInfo.navwiz_version}</h3>
        <p>
          <Perm perms={['system.update_software']}>
            <a href="/config/software-update" class="variant-filled btn">
              Check for updates...
            </a>
          </Perm>
          <Perm perms={['system.update_hardware_plugin']}>
            <a href="/config/hardware-plugin" class="variant-filled btn">
              Manage hardware plugin...
            </a>
          </Perm>
          <Perm perms={['system.update_software']}>
            <a href="/config/software-patch" class="variant-filled btn">Manage patch...</a>
          </Perm>
        </p>
        <p>NavWiz is the software that powers all the AGVs developed by DF Automation.</p>
        <p>It controls all the operations for the AGV, which include the following:</p>
        <ul class="list-disc pl-8">
          <li>executing task;</li>
          <li>navigating to the destination safely;</li>
          <li>providing a panel for user interaction; and</li>
          <li>providing a backend to configure the map and task templates.</li>
        </ul>
        <hr />
        <p>{@html licenseStatus}</p>
        <Perm perms={['system.view_system_panel', 'system.change_license']}>
          <p>
            <a href="/config/license" class="variant-filled btn">Change license key</a>
          </p>
        </Perm>
      </section>
      <footer class="card-footer"></footer>
    </div>

    <div class="card h-full w-full shadow-lg">
      <header class="card-header p-3 px-7 font-semibold">Help and Support</header>
      <hr />
      <section class="p-7 space-y-2">
        <div class="grid grid-cols-3 gap-4">
          <span>Remote Assistance:</span>
          <button
            type="button"
            class="variant-filled-success btn"
            on:click={remoteConnect}
            disabled={remoteConnected}>Start</button>
          <button
            type="button"
            class="variant-filled-error btn"
            on:click={remoteDisconnect}
            disabled={!remoteConnected}>Stop</button>
        </div>

      </section>
      <footer class="card-footer"></footer>
    </div>

    <div class="card h-full w-full shadow-lg">
      <header class="card-header p-3 px-7 font-semibold">Contact Us</header>
      <hr />
      <section class="contact-us space-y-3 p-7">
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        {@html $whiteLabel.address_label || ''}
      </section>
      <footer class="card-footer"></footer>
    </div>
  </div>
</ConfigLayout>

<style lang="postcss">
  .contact-us :global(h3) {
    @apply text-3xl;
  }
</style>
