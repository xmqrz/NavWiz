<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import { SlideToggle } from '@skeletonlabs/skeleton';
  import { sortBy } from 'lodash-es';

  import { agv05WiFi } from '$lib/shared/services/user-panel/agv05-wifi.service.js';

  const WIFILIST_TIMER = 5000; // 5 sec

  // Props
  // svelte-ignore unused-export-let
  export let parent;
  export let wifi;
  export let wnics;
  export let config;
  export let updateStatus;

  const modalStore = getModalStore();

  let loading = false;
  let adhocLoading = false;

  let mount = true;
  let wifiListTimeout;

  function updateWifiListTimer(state) {
    if (!wifiListTimeout && state) {
      wifiListTimeout = setTimeout(updateWifiList, 0);
    } else if (wifiListTimeout && !state) {
      clearTimeout(wifiListTimeout);
      wifiListTimeout = undefined;
    }
  }

  $: updateWifiListTimer($wnics.state);

  onMount(() => {
    mount = true;

    return () => {
      if (wifiListTimeout) {
        clearTimeout(wifiListTimeout);
        wifiListTimeout = undefined;
      }
      mount = false;
    };
  });

  function updateWifiList() {
    if (!mount || !$wnics.state) {
      return;
    }
    loading = true;
    agv05WiFi
      .WIFILIST()
      .then(function (data) {
        // To hide hidden ssid
        let filteredList = [];
        for (let w of data.ssid_list) {
          if (w['ssid'] === '') {
            continue;
          }
          filteredList.push(w);
        }
        $wnics.wifiList = sortBy(filteredList, 'sig').reverse();
        if (data.ip_cfg) {
          if (data.ip_cfg['source'] === 'dhcp') {
            $config.staticOpt = false;
          } else if (data.ip_cfg['source'] === 'static') {
            $config.staticOpt = true;
          }
          $config.ipv4 = data.ip_cfg['address'];
          $config.netmask = data.ip_cfg['netmask'];
          $config.gateway = data.ip_cfg['gateway'];

          let dnsNameserver = data.ip_cfg['dns-nameservers'];
          if (Array.isArray(dnsNameserver)) {
            $config.dnsNameserver = dnsNameserver[0];
          } else {
            $config.dnsNameserver = dnsNameserver;
          }
        }
      })
      .finally(function () {
        wifiListTimeout = undefined;
        loading = false;
        if (!mount || !$wnics.state) {
          return;
        }

        wifiListTimeout = setTimeout(updateWifiList, WIFILIST_TIMER);
      });
  }

  function onSetIp() {
    if ($modalStore[0].response) {
      $modalStore[0].response({
        action: 'setIP'
      });
    }
    modalStore.close();
  }

  function onConnectHiddenSSID() {
    if ($modalStore[0].response) {
      $modalStore[0].response({
        action: 'connectHiddenSSID'
      });
    }
    modalStore.close();
  }

  function onConnectSSID(ssid, strength, encryption, status) {
    if ($modalStore[0].response) {
      $modalStore[0].response({
        action: 'connectSSID',
        ssid,
        strength,
        encryption,
        status
      });
    }
    modalStore.close();
  }

  function onWifiError(error) {
    if ($modalStore[0].response) {
      $modalStore[0].response({
        action: 'wifiError',
        error
      });
    }
    modalStore.close();
  }

  function onAdhocChange() {
    adhocLoading = true;
    let adhoc = $wnics.adhoc;
    agv05WiFi
      .ADHOC(adhoc)
      .then(function (data) {
        if (data.cancelled) {
          return;
        }
        if (data.result !== true) {
          $wnics.adhoc = !adhoc;
          onWifiError(data.error);
        }
      })
      .finally(function () {
        adhocLoading = false;
      });
  }

  function onWnicsChange() {
    loading = true;
    let state = $wnics.state;
    agv05WiFi
      .WNICS(state)
      .then(function (data) {
        if (data.cancelled) {
          return true;
        }
        if (data.result !== true) {
          $wnics.state = !state;
          onWifiError(data.error);
        }
        return new Promise((resolve) => setTimeout(resolve, 5000));
      })
      .then(function (skip) {
        if (skip) {
          return;
        }
        // TODO: this trigger elevated twice.
        return updateStatus();
      })
      .finally(function () {
        loading = false;
      });
  }
</script>

{#if $modalStore[0]}
  <div class="card max-h-[90vh] w-96 space-y-2 self-center overflow-auto p-4">
    <div class="center card-header" align="center">
      {#if $wifi.ssid != ''}
        <span class="text-lg">{$wifi.ssid}</span>
      {:else}
        <span class="text-lg">{$wifi.status}</span>
      {/if}
    </div>
    <hr />
    <div class="card grid grid-cols-4 bg-white p-4">
      <div class="list col-span-3">
        <div>IP : {$wifi.ip}</div>
        <div>MAC : {$wifi.mac}</div>
      </div>
      <button type="button" class="btn p-4" on:click={onSetIp}>
        <i class="fa-solid fa-screwdriver-wrench fa-xl"></i>
      </button>
    </div>
    <div class="card list bg-white p-4">
      <div class="grid grid-cols-4">
        <span class="col-span-3 flex justify-between">
          Hotspot
          {#if adhocLoading}
            <div class="self-center px-2">
              <ProgressRadial width="w-5" />
            </div>
          {/if}
        </span>
        <div class="flex justify-center">
          <SlideToggle
            name="HotspotSwitch"
            disabled={$wnics.state == false}
            bind:checked={$wnics.adhoc}
            on:change={onAdhocChange} />
        </div>
      </div>
      <div class="grid grid-cols-4">
        <span class="col-span-3 flex justify-between">
          Wi-Fi
          {#if loading}
            <div class="self-center px-2">
              <ProgressRadial width="w-5" />
            </div>
          {/if}
        </span>
        <div class="flex justify-center">
          <SlideToggle
            name="WifiSwitch"
            bind:checked={$wnics.state}
            on:change={onWnicsChange} />
        </div>
      </div>
    </div>
    {#if $wnics.state}
      <div class="card bg-white p-4">
        <div class="grid grid-cols-4 items-center">
          <span class="col-span-3">Add Network</span>
          <button type="button" class="btn p-4" on:click={onConnectHiddenSSID}>
            <i class="fa-solid fa-square-plus fa-xl"></i>
          </button>
        </div>
      </div>
      <div class="card bg-white p-2">
        <div class="text-slate-600" align="center">AVAILABLE</div>
        <hr />
        {#if !$wnics.wifiList.length}
          <p class="pl-2 italic">No Wi-Fi Available</p>
        {:else}
          <dl class="list-dl">
            {#each $wnics.wifiList as w}
              <!-- svelte-ignore a11y-no-static-element-interactions -->
              <!-- svelte-ignore a11y-click-events-have-key-events -->
              <div
                on:click={() => onConnectSSID(w['ssid'], w['sig'], w['enc'], w['status'])}
                class="cursor-pointer hover:shadow-md">
                <span class="badge">
                  <svg viewbox="0 0 24 20" class="mr-0.5">
                    <g transform="scale(0.6)">
                      <path
                        class="wifi-bar"
                        class:active={w['sig'] >= 20}
                        d="M17.98 28.12c0-1.1.9-2.02 2.02-2.02s2.02.9 2.02 2.02-.9 2.02-2.02 2.02-2.02-.9-2.02-2.02" />
                      <path
                        class="wifi-bar"
                        class:active={w['sig'] >= 50}
                        d="M20 18.02c3.34 0 6.37 1.36 8.57 3.55l-2.86 2.86c-1.45-1.46-3.47-2.37-5.7-2.37s-4.25.9-5.7 2.37l-2.87-2.86c2.2-2.2 5.23-3.55 8.57-3.55" />
                      <path
                        class="wifi-bar"
                        class:active={w['sig'] >= 70}
                        d="M5.7 15.86c3.83-3.82 8.9-5.92 14.3-5.92s10.47 2.1 14.3 5.92l-2.87 2.85C28.38 15.67 24.33 14 20 14s-8.38 1.68-11.43 4.73L5.7 15.87" />
                      <path
                        class="wifi-bar"
                        class:active={w['sig'] >= 90}
                        d="M31 4.08c3.38 1.43 6.4 3.47 9 6.06L37.14 13C32.56 8.42 26.48 5.9 20 5.9S7.44 8.42 2.86 13L0 10.14c2.6-2.6 5.62-4.63 9-6.06 3.48-1.47 7.18-2.22 11-2.22s7.52.74 11 2.22" />
                    </g>
                  </svg>
                </span>
                <span class="flex-auto">
                  <dt>{w['ssid']}</dt>
                  <dd class="text-xs text-slate-400">Signal Strength: {w['sig']}</dd>
                </span>
              </div>
            {/each}
          </dl>
        {/if}
      </div>
    {/if}
  </div>
{/if}

<style>
  svg {
    width: 1em;
    font: inherit;
    float: left;
    font-size: 24px;
    stroke: black;
    fill: black;
  }

  .wifi-bar {
    opacity: 0.3;
  }

  .wifi-bar.active {
    opacity: 1;
  }
</style>
