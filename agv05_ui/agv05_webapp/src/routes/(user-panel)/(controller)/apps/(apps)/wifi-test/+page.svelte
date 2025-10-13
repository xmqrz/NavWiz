<script>
  import { onMount, getContext } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';

  const modalStore = getModalStore();
  const mainController = getContext('mainController');
  const wifiTest = mainController.moduleService.getSub('wifi-test');

  let data = {};
  let showDownloadLink = !window.agvPanelToken;
  let recording = false;

  function startRecording() {
    wifiTest.publish({
      command: 'start_recording'
    });
  }

  function stopRecording() {
    wifiTest.publish({
      command: 'stop_recording'
    });
  }

  function getTimestamp(data) {
    if (data.timestamp) {
      let d = new Date(data.timestamp * 1000);
      return d.toString();
    } else {
      return '-';
    }
  }

  function wifiTestCallback(d) {
    if (d.command === 'status') {
      recording = d.status === 'recording';
    } else if (d.command === 'data') {
      data = d.data;
    }
  }

  onMount(() => {
    wifiTest.subscribe(wifiTestCallback);
    // allow time for subscribe to happen first.
    setTimeout(function () {
      wifiTest.publish({
        command: 'status'
      });
    }, 100);

    return () => {
      wifiTest.unsubscribe(wifiTestCallback);
    };
  });

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Wifi Test" ?',
      response: (r) => {
        if (r) {
          wifiTest.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  const cardStyle = 'card shadow-lg';
  const cardHeaderStyle = 'card-header w-full pb-2 pt-2 text-center text-md tracking-widest';
  const cardBodyStyle = 'px-2 py-2 pt-2';
</script>

<AppLayout title="Wifi Test" moduleID="wifi-test" on:close={shutdown}>
  <div class="grow overflow-auto p-4">
    <div class="space-y-4">
      <div>
        Timestamp: <span>{getTimestamp(data)}</span>
      </div>
      <div class="{cardStyle} grid grid-cols-4 divide-x divide-solid divide-black">
        <header class="{cardHeaderStyle} flex content-center items-center justify-center">
          Data Recording
          <i
            class="fa-solid fa-circle ml-5 text-red-500 opacity-0"
            class:fa-beat-fade={recording}></i>
        </header>
        <section class="{cardBodyStyle} col-span-3">
          <div class="flex justify-center gap-4">
            <button
              type="button"
              class="variant-filled-warning btn"
              on:click={startRecording}
              disabled={recording}>Start</button>
            <button
              type="button"
              class="variant-filled btn"
              on:click={stopRecording}
              disabled={!recording}>
              Stop
            </button>
            {#if showDownloadLink}
              <a href="/logs/wifi" target="_blank">
                <button type="button" class="btn text-sky-400">Downloads</button>
              </a>
            {/if}
          </div>
        </section>
      </div>
      <div class="grid grid-cols-2 gap-4">
        <div class={cardStyle}>
          <header class={cardHeaderStyle}>Wifi</header>
          <hr />
          <section class="{cardBodyStyle} col-span-3">
            {#each data.wifi || [] as kv}
              <div class="grid grid-cols-2">
                <span>{kv.key}</span>
                <span>{kv.value}</span>
              </div>
            {/each}
          </section>
        </div>
        <div class={cardStyle}>
          <header class={cardHeaderStyle}>DFleet</header>
          <hr />
          <section class="{cardBodyStyle} col-span-3 min-h-20">
            {#each data.fms || [] as kv}
              <div class="grid grid-cols-2">
                <span>{kv.key}</span>
                <span>{kv.value}</span>
              </div>
            {/each}
          </section>
        </div>
      </div>
    </div>
  </div>
</AppLayout>
