<script>
  import { onMount, getContext } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';

  const mainController = getContext('mainController');
  const devMode = mainController.moduleService.getSub('dev-mode');
  const modalStore = getModalStore();

  let running = false;
  let serverVersion = null;

  function devModeCallback(data) {
    if (data.command === 'status') {
      running = data.status === 'running';
      serverVersion = data.server_version;
    }
  }

  onMount(() => {
    devMode.subscribe(devModeCallback);
    // allow time for subscribe to happen first.
    setTimeout(function () {
      devMode.publish({
        command: 'status'
      });
    }, 100);

    return () => {
      devMode.unsubscribe(devModeCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Development Mode" ?',
      response: (r) => {
        if (r) {
          devMode.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }
</script>

<AppLayout title="Development Mode" moduleID="dev-mode" on:close={shutdown}>
  <div class="min-h-24 grow overflow-auto p-4">
    {#if running}
      <div>Development Mode API Server has started.</div>
      <label>
        <span>Server Version</span>
        <input type="text" readonly="readonly" bind:value={serverVersion} />
      </label>
    {:else}
      <div>
        Development Mode API Server fails to start. Please verify that the plugin has been
        installed.
      </div>
    {/if}
  </div>
</AppLayout>
