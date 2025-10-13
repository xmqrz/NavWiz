<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';
  import { alertModal } from '$lib/modal-service.js';
  import { fmsChannel } from 'stores/sock/index.js';

  const mainController = getContext('mainController');
  const homing = mainController.moduleService.getSub('homing');
  const modalStore = getModalStore();

  let message = '';
  let running = false;
  let stations = [];

  /* fms status update */
  let fmsStatus = null;
  let fmsBroken = false;
  $: isFmsMode = !!fmsStatus;

  function startHoming(station) {
    homing.publish({
      command: 'start_homing',
      station: station
    });
  }

  function stopHoming() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stop homing?',
      response: (r) => {
        if (r) {
          homing.publish({
            command: 'stop_homing'
          });
        }
      }
    };
    modalStore.trigger(modal);
  }

  function homingCallback(data) {
    if (data.command === 'list_all') {
      if (Array.isArray(data.stations)) {
        stations = data.stations;
      }
    } else if (data.command === 'status') {
      running = data.status === 'running';
      message = data.text;
    } else if (data.command === 'error') {
      modalStore.trigger(alertModal('Homing Error', data.error));
    } else if (data.command === 'outcome') {
      modalStore.trigger(alertModal('Homing Outcome', data.outcome));
    }
  }

  function fmsCallback(data) {
    if (data.id === 'status') {
      fmsStatus = data.value.replace(/\n/g, '<br/>');
      fmsBroken = data.broken;
    }
  }

  onMount(() => {
    homing.subscribe(homingCallback);
    fmsChannel.subscribe(fmsCallback);
    setTimeout(function () {
      homing.publish({
        command: 'list_all'
      });
      homing.publish({
        command: 'status'
      });
    }, 100);

    return () => {
      homing.unsubscribe(homingCallback);
      fmsChannel.unsubscribe(fmsCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Homing" ?',
      response: (r) => {
        if (r) {
          homing.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }
</script>

<AppLayout title="Homing" moduleID="homing" on:close={shutdown}>
  <div class="grow overflow-auto p-4">
    <div>
      Please park AGV at any of the following stations, and select the station to begin homing.
    </div>
    <div class="my-4" style="height: 70%; overflow: auto">
      {#if isFmsMode && fmsBroken}
        <div>
          <!-- eslint-disable-next-line svelte/no-at-html-tags -->
          DFleet error: <span>{@html fmsStatus}</span>
        </div>
      {:else}
        {#each stations as _st, stIndex}
          {#if stIndex % 4 === 0}
            <div
              class="mb-2 mt-2 grid grid-flow-row grid-cols-1 gap-2 md:grid-cols-2 lg:grid-cols-4">
              {#each [0, 1, 2, 3] as i}
                {#if stations[stIndex + i]}
                  <button
                    type="button"
                    class="variant-ghost-surface btn btn-lg"
                    on:click={() => startHoming(stations[stIndex + i])}
                    disabled={running}>
                    <span class="truncate">
                      {stations[stIndex + i]}
                    </span>
                  </button>
                {/if}
              {/each}
            </div>
          {/if}
        {/each}
      {/if}
    </div>
    {#if running}
      <div>
        <button type="button" class="variant-filled-error btn btn-sm" on:click={stopHoming}>
          <i class="fa-solid fa-exclamation mr-2"></i>Stop Homing
        </button>
      </div>
    {/if}
    {#if running && message}
      <div>{message}</div>
    {/if}
  </div>
</AppLayout>
