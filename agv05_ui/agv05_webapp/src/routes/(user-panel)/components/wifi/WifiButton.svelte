<script>
  import { writable } from 'svelte/store';
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';

  import { notificationChannel } from 'stores/sock/index.js';
  import { agv05WiFi } from '$lib/shared/services/user-panel/agv05-wifi.service.js';
  import { agv05WiFiSetIP } from '$lib/shared/services/user-panel/agv05-wifisetip.service.js';
  import { agv05WiFiCon } from '$lib/shared/services/user-panel/agv05-wificon.service.js';
  import { alertModal } from '$lib/modal-service.js';
  import ModalWifiConfig from './modal/ModalWifiConfig.svelte';
  import ModalStaticIpConfig from './modal/ModalStaticIpConfig.svelte';
  import ModalHSSID from './modal/ModalHSSID.svelte';
  import ModalWifiConnect from './modal/ModalWifiConnect.svelte';

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let wifiModal;

  let wifi = writable({
    status: 'Not Connected',
    adhoc: false,
    ip: '',
    ssid: '',
    siglvl: -200,
    mac: ''
  });

  let wnics = writable({
    state: false,
    adhoc: false,
    wifiList: []
  });

  let config = writable({
    staticOpt: false,
    ipv4: '',
    netmask: '',
    gateway: '',
    dnsNameserver: ''
  });

  function closeWifiModal() {
    if (wifiModal) {
      modalStore.close(wifiModal);
      wifiModal = undefined;
    }
  }

  function updateStatus() {
    return agv05WiFi.STATUS().then(function (data) {
      if (data.cancelled) {
        return data;
      }
      if (data.result !== true) {
        // TODO: we unload current modal then add this one instedad... then trigger the wifimodal again??
        modalStore.priorityTrigger(alertModal('Wi-Fi error: ' + data.error));
      }
      if (
        data.message['wnics'] === 'connected' ||
        data.message['wnics'] === 'on' ||
        data.message['wnics'] === 'disconnected'
      ) {
        $wnics.state = true;
      } else if (data.message['wnics'] === 'off') {
        $wnics.state = false;
      }
      if (data.message['adhoc'] === 'on') {
        $wnics.adhoc = true;
      } else if (data.message['wnics'] === 'off') {
        $wnics.adhoc = false;
        $wnics.wifiList = [];
      }
      return data;
    });
  }

  function setIP(staticmode, ip, netmask, gateway, dnsNameserver) {
    return agv05WiFiSetIP
      .SETIP(staticmode, ip, netmask, gateway, dnsNameserver)
      .then(function (data) {
        if (data.cancelled) {
          return data;
        }
        if (data.result !== true) {
          // TODO: we unload current modal then add this one instedad... then trigger the wifimodal again??
          modalStore.priorityTrigger(alertModal('Wi-Fi error', data.error));
        }
        if (data.message === 'success') {
          toastStore.trigger({
            message: 'IP configuration saved.',
            timeout: 3000,
            hoverable: true
          });
        }
      });
  }

  function connectwifi(ssid, user, pass, enc, eap, hidden = false) {
    agv05WiFiCon.CONNECT(ssid, user, pass, enc, eap, hidden).then(function (data) {
      if (data.cancelled) {
        return data;
      }
      if (data.result !== true) {
        modalStore.priorityTrigger(alertModal('Wi-Fi error', data.error));
      }
      if (data.message === 'COMPLETED') {
        toastStore.trigger({
          message: 'Wi-Fi connection successful.',
          timeout: 3000,
          hoverable: true
        });
      } else if (
        data.message === 'ASSOCIATED' ||
        data.message === 'ASSOCIATING' ||
        data.message === '4WAY_HANDSHAKE' ||
        data.message === 'SCANNING'
      ) {
        modalStore.priorityTrigger(
          alertModal(`Connecting to "${ssid}" is taking too long time.`)
        );
      } else if (data.message === 'DISCONNECTED') {
        modalStore.priorityTrigger(alertModal('Failed to connect the network.'));
      } else {
        modalStore.priorityTrigger(alertModal('Wi-Fi error', data.message));
      }
    });
  }

  function forgetNetwork(ssid) {
    const modal = {
      type: 'confirm',
      title: `Confirm to forget "${ssid}" Wi-Fi network?`,
      response: (r) => {
        if (r) {
          agv05WiFiCon.FORGET(ssid).then(function (data) {
            if (data.cancelled) {
              return data;
            }
            if (data.result !== true) {
              modalStore.priorityTrigger(alertModal('Wi-Fi error', data.error));
            }
            if (data.message === 'success') {
              toastStore.trigger({
                message: 'Forget Wi-Fi successful.',
                timeout: 3000,
                hoverable: true
              });
            }
          });
        }
      }
    };
    modalStore.trigger(modal);
  }

  function configWifi(skipUpdateStatus) {
    closeWifiModal();
    wifiModal = {
      type: 'component',
      component: {
        ref: ModalWifiConfig,
        props: {
          wifi,
          wnics,
          config,
          updateStatus
        }
      },
      response: onConfigResponse
    };
    modalStore.trigger(wifiModal);

    if (skipUpdateStatus) {
      return;
    }

    updateStatus().then(function (data) {
      if (data.cancelled) {
        closeWifiModal();
      }
    });
  }

  function onConfigResponse(r) {
    if (!r || !r.action) {
      return;
    }

    if (r.action === 'wifiError') {
      modalStore.priorityTrigger(alertModal('Wi-Fi error', r.error));
      configWifi(true);
    } else if (r.action === 'setIP') {
      wifiModal = {
        type: 'component',
        component: {
          ref: ModalStaticIpConfig,
          props: {
            config
          }
        },
        response: (r) => {
          if (r) {
            setIP(r.staticOpt, r.ipv4, r.netmask, r.gateway, r.dnsNameserver);
          }
          wifiModal = undefined;
          configWifi(true);
        }
      };
      modalStore.trigger(wifiModal);
    } else if (r.action === 'connectHiddenSSID') {
      wifiModal = {
        type: 'component',
        component: {
          ref: ModalHSSID
        },
        response: (r) => {
          if (r) {
            connectwifi(r.ssid, r.identity, r.password, r.encrypt, r.eap, true);
          }
          wifiModal = undefined;
          configWifi(true);
        }
      };
      modalStore.trigger(wifiModal);
    } else if (r.action === 'connectSSID') {
      wifiModal = {
        type: 'component',
        component: {
          ref: ModalWifiConnect,
          props: {
            ssid: r.ssid,
            strength: r.strength,
            encryption: r.encryption,
            status: r.status
          }
        },
        response: (r) => {
          if (r) {
            if (r.action === 'connect') {
              connectwifi(r.ssid, r.identity, r.password, r.encrypt, r.eap);
            } else if (r.action === 'forget') {
              forgetNetwork(r.ssid);
            }
          }
          wifiModal = undefined;
          configWifi(true);
        }
      };
      modalStore.trigger(wifiModal);
    }
  }

  function notifyCb(data) {
    if (data.id === 'adhoc') {
      $wifi.adhoc = data.value;
    } else if (data.id === 'wifi') {
      $wifi.status = data.valuestate;
      $wifi.ip = data.valueip;
      $wifi.ssid = data.ssid;
      let signallevel = parseInt(data.signallevel);
      if (!signallevel) {
        signallevel = data.valuestate === 'connected' ? 0 : -128;
      } else if (signallevel > 0) {
        signallevel -= 128; // signallevel comes in percentage format
      }
      $wifi.siglvl = signallevel;
      $wifi.mac = data.mac;
    }
  }

  onMount(() => {
    notificationChannel.subscribe(notifyCb);
    return () => {
      notificationChannel.unsubscribe(notifyCb);
    };
  });
</script>

<button
  type="button"
  class="variant-filled-primary bg-primary-400-500-token btn h-[42px] px-1 md:px-5"
  on:click={() => configWifi()}>
  {#if $wifi.adhoc}
    <i class="fa-solid fa-tower-cell m-1 text-black"></i>
    {#if $wifi.ip}
      <span class="hidden text-sm text-black md:inline">
        {$wifi.ip}
      </span>
    {/if}
  {:else}
    {#if $wifi.ssid}
      <svg viewbox="0 0 24 20" class="ml-2 mr-2.5">
        <g transform="scale(0.6)">
          <path
            class="wifi-bar"
            class:active={$wifi.siglvl >= -100}
            d="M17.98 28.12c0-1.1.9-2.02 2.02-2.02s2.02.9 2.02 2.02-.9 2.02-2.02 2.02-2.02-.9-2.02-2.02" />
          <path
            class="wifi-bar"
            class:active={$wifi.siglvl >= -90}
            d="M20 18.02c3.34 0 6.37 1.36 8.57 3.55l-2.86 2.86c-1.45-1.46-3.47-2.37-5.7-2.37s-4.25.9-5.7 2.37l-2.87-2.86c2.2-2.2 5.23-3.55 8.57-3.55" />
          <path
            class="wifi-bar"
            class:active={$wifi.siglvl >= -80}
            d="M5.7 15.86c3.83-3.82 8.9-5.92 14.3-5.92s10.47 2.1 14.3 5.92l-2.87 2.85C28.38 15.67 24.33 14 20 14s-8.38 1.68-11.43 4.73L5.7 15.87" />
          <path
            class="wifi-bar"
            class:active={$wifi.siglvl >= -67}
            d="M31 4.08c3.38 1.43 6.4 3.47 9 6.06L37.14 13C32.56 8.42 26.48 5.9 20 5.9S7.44 8.42 2.86 13L0 10.14c2.6-2.6 5.62-4.63 9-6.06 3.48-1.47 7.18-2.22 11-2.22s7.52.74 11 2.22" />
        </g>
      </svg>
    {:else}
      <span class="fa-stack text-black">
        <i class="fa-solid fa-wifi fa-stack-1x"></i>
        <i class="fa-solid fa-slash fa-stack-1x"></i>
      </span>
    {/if}
    {#if $wifi.ip}
      <span class="hidden text-sm text-black md:inline">
        {$wifi.ip}
      </span>
    {/if}
  {/if}
</button>

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
