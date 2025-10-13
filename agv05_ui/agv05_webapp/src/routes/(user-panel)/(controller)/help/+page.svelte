<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onDestroy, onMount } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import QRCode from 'qrcode';

  import { alertModal } from '$lib/modal-service.js';
  import { getAPI } from '$lib/utils';
  import { remoteAssistChannel } from 'stores/sock';
  import { version } from 'stores/version.js';
  import { whiteLabel } from 'stores/white-label.js';

  let currentY = 0;
  let pulling = false;
  let refreshing = false;
  let remoteConnected = false;
  let resistance = 0.3;
  let shouldRefresh = false;
  let showExtras = false;
  let startY = 0;
  let translateY = 0;
  let touched = false;

  const cardStyle = 'card shadow-lg';
  const cardHeaderStyle =
    'card-header w-full py-3 text-center text-md tracking-widest font-semibold';
  const cardBodyStyle = 'p-7 pt-4';
  const modalStore = getModalStore();
  const qr = getAPI('/agv').then((agv) => {
    if (agv?.uuid) {
      const link = `https://hub.dfautomation.com/w/products/m/${agv.uuid}`;
      return QRCode.toDataURL(link).then((image) => ({ link, image }));
    }
  });

  function toggleShowExtras() {
    //   $scope.$broadcast('scroll.refreshComplete');
    showExtras = !showExtras;
  }

  function remoteConnect() {
    remoteAssistChannel.publish({
      id: 'remote_connect'
    });
  }

  function remoteDisconnect() {
    remoteAssistChannel.publish({
      id: 'remote_disconnect'
    });
  }

  function doRefresh() {
    window.location.reload();
  }

  let tapStart;
  let tapCount;
  function hiddenTap() {
    let now = new Date();
    if (!tapStart || now - tapStart > 3000) {
      tapStart = now;
      tapCount = 1;
    } else if (++tapCount === 5) {
      tapStart = null;
      toggleShowExtras();
    }
  }

  const touchStart = (event) => {
    if (event.type === 'touchstart') {
      startY = event.touches[0].clientY;
    } else if (event.type === 'mousedown') {
      startY = event.clientY;
    }
    touched = true;
  };

  const touchMove = (event) => {
    if (!touched) return;

    if (event.type === 'touchmove') {
      currentY = event.touches[0].clientY;
    } else if (event.type === 'mousemove') {
      currentY = event.clientY;
    }

    let touchDifference = currentY - startY;
    shouldRefresh = touchDifference > 60;
    pulling = touchDifference > 20;
    translateY = touchDifference * resistance;
  };

  function onRefresh() {
    refreshing = true;
    setTimeout(() => {
      toggleShowExtras();
      refreshing = false;
      pulling = false;
    }, 700);
  }

  const touchEnd = () => {
    translateY = 0;
    pulling = shouldRefresh;
    if (shouldRefresh) {
      onRefresh();
    }
    shouldRefresh = false;
    touched = false;
  };

  function remoteAssistCallback(data) {
    if (data.id === 'status') {
      remoteConnected = data.value;
    } else if (data.id === 'error') {
      modalStore.trigger(alertModal('Failed to start remote assistance.'));
    }
  }
  onMount(() => {
    showExtras = false;
    remoteAssistChannel.subscribe(remoteAssistCallback);
  });

  onDestroy(() => {
    remoteAssistChannel.unsubscribe(remoteAssistCallback);
  });
</script>

<!-- svelte-ignore a11y-no-static-element-interactions -->
<div
  class="space-y-7"
  on:touchstart={touchStart}
  on:touchmove={touchMove}
  on:touchend={touchEnd}
  on:mousedown={touchStart}
  on:mousemove={touchMove}
  on:mouseup={touchEnd}>
  {#if pulling}
    <div class="flex justify-center py-4" style="margin-top: {translateY}px;">
      {#if refreshing}
        <ProgressRadial width="w-6" />
      {:else}
        <span class="mr-4">
          {#if shouldRefresh}
            <i class="fa-solid fa-arrow-up"></i>
          {:else}
            <i class="fa-solid fa-arrow-down"></i>
          {/if}
        </span>
        Pull to toggle additional options...
      {/if}
    </div>
  {/if}
  <div class="max-w-3xl space-y-7">
    <div class={cardStyle}>
      <header class={cardHeaderStyle}>Agv Version</header>
      <hr />
      <section class={cardBodyStyle}>
        <div>
          {#await qr then qr}
            {#if qr}
              <div class="mb-2 sm:float-right">
                <a href={qr.link} target="_blank" title="View in DF Hub">
                  <img src={qr.image} />
                </a>
              </div>
            {/if}
          {/await}
          {#if $version && $version.info}
            {@const info = $version.info}
            <div class="grid grid-cols-4 md:grid-cols-5">
              <div class="col-span-2">AGV Model</div>
              <div class="col-span-2">{info.agv_model}</div>
              <div class="col-span-2">Serial Number</div>
              <div class="col-span-2">{info.serial_number}</div>
              <div class="col-span-2">Manufacture Date</div>
              <div class="col-span-2">{info.manufacture_date}</div>
              <div class="col-span-2">Next P.M. Due</div>
              <div class="col-span-2">{info.next_pm_due}</div>
              <div class="col-span-2">Next P.M. Mileage</div>
              <div class="col-span-2">{info.next_pm_mileage} m</div>
              <div class="col-span-2">Current Mileage</div>
              <div class="col-span-2">{info.current_mileage} m</div>
              <div class="col-span-2">System Version</div>
              <div class="col-span-2">{info.system_version}</div>
              <div class="col-span-2">Plugin Version</div>
              <div class="col-span-2 whitespace-pre-line">{info.plugin_version}</div>
              {#each info.hw_info || [] as [k, v]}
                <div class="col-span-2">{k}</div>
                <div class="col-span-2">{v}</div>
              {/each}
            </div>
          {/if}
        </div>
      </section>
    </div>

    {#if showExtras}
      <div class={cardStyle}>
        <header class={cardHeaderStyle}>Help and Support</header>
        <hr />
        <section class="{cardBodyStyle} space-y-2">
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
          {#if window.agvPanelToken}
            <div>
              <button type="button" class="variant-filled btn btn-sm" on:click={doRefresh}>
                <i class="fa-solid fa-rotate-right mr-2"></i> Refresh Panel
              </button>
            </div>
          {/if}
        </section>
      </div>
    {/if}

    <div class={cardStyle} on:release|stopPropagation={hiddenTap}>
      <header class={cardHeaderStyle}>Contact Us</header>
      <hr />
      <section class={cardBodyStyle}>
        <div>
          <!-- eslint-disable-next-line svelte/no-at-html-tags -->
          {@html $whiteLabel.address_label}
        </div>
      </section>
    </div>
  </div>
</div>
