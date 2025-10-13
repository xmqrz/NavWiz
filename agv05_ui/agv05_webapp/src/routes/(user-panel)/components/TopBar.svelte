<script>
  import { AppBar, getModalStore } from '@skeletonlabs/skeleton';
  import { onDestroy, getContext } from 'svelte';
  import { derived } from 'svelte/store';
  import { t } from '$lib/translations';

  import { agv05Boot } from '$lib/shared/services/user-panel/agv05-boot.service.js';
  import { alertModal, httpErrorModal } from '$lib/modal-service.js';

  import { fmsChannel, notificationChannel } from 'stores/sock/index.js';
  import { popupType, robotName, robotRunning } from 'stores/states.js';
  import { syncClock } from 'stores/server-clock.js';
  import Perm from 'components/Perm.svelte';
  import TimeDisplay from 'components/TimeDisplay.svelte';
  import UserMenu from 'components/UserMenu.svelte';
  import WifiButton from './wifi/WifiButton.svelte';

  const modalStore = getModalStore();
  const mainController = getContext('mainController');
  const logo = `${API_URL}/white-label/logo`;

  const safetyNotification = derived(popupType, ($popupType, set) => {
    if ($popupType !== 'safety') {
      set(false);
      return;
    }
    set(true);
    const timer = setTimeout(() => set(false), 500);
    return () => clearTimeout(timer);
  });
  const hasTracker = mainController.downtimeTrackerService.hasTracker();

  let battery = 90;
  let batteryCharging = false;
  let batteryChargingManual = false;
  let countdown = 120;
  let diskSpaceRemaining = 0;
  let fmsBroken = false;
  let fmsStatus = null;
  let diskSpaceWarning = '';
  let maintenanceWarning = '';

  $: isFmsMode = !!fmsStatus;

  notificationChannel.subscribe(notifyCb);
  fmsChannel.subscribe(fmsCallback);
  onDestroy(() => {
    notificationChannel.unsubscribe(notifyCb);
    fmsChannel.unsubscribe(fmsCallback);
  });

  function notifyCb(data) {
    if (data.id === 'server_time') {
      syncClock(data);
    } else if (data.id === 'robot_running') {
      if (data.value === 0) {
        countdown = null;
      }
      fmsStatus = null;
      fmsBroken = false;
    } else if (data.id === 'battery') {
      battery = data.value;
      batteryCharging = data.state;
      batteryChargingManual = data.manual;
    } else if (data.id === 'disk_space') {
      diskSpaceRemaining = data.remaining;
      if (data.warning) {
        diskSpaceWarning = `Free space remaining: ${diskSpaceRemaining}%<br>Please perform hard disk maintenance.`;
      } else {
        diskSpaceWarning = '';
      }
    } else if (data.id === 'maintenance') {
      maintenanceWarning = data.warning || '';
      if (maintenanceWarning) {
        showMaintenanceWarning();
      }
    } else if (data.id === 'countdown') {
      countdown = data.value;
    }
  }

  function fmsCallback(data) {
    if (data.id === 'status') {
      fmsStatus = data.value.replace(/\n/g, '<br/>');
      fmsBroken = data.broken;
    }
  }

  function isCountdownValid() {
    return !$robotRunning && countdown !== null;
  }

  function extendCountdown() {
    if (!isCountdownValid()) {
      return;
    }
    const modal = {
      type: 'confirm',
      title: 'Extend countdown to 30 mins?',
      position: 'items-center',
      response: (r) => {
        if (!r || !isCountdownValid()) {
          return;
        }
        agv05Boot
          .extendCountdown()
          .then(function (data) {
            if (data.cancelled) {
              return;
            }
            if (data.result !== true) {
              modalStore.trigger(
                alertModal('Failed to extend countdown', data.message.replace(/\n/g, '<br/>'))
              );
            }
          })
          .catch(function (http) {
            modalStore.trigger(httpErrorModal('Failed to extend countdown.', http));
          });
      }
    };
    modalStore.trigger(modal);
  }

  function showDiskSpaceWarning() {
    modalStore.trigger(alertModal('Low Disk Space', diskSpaceWarning));
  }

  function showMaintenanceWarning() {
    modalStore.trigger(alertModal('Preventive Maintenance Due', maintenanceWarning));
  }

  function getBatteryIcon(isManualCharging, battery) {
    if (isManualCharging) {
      return 'fa-plug';
    } else if (battery >= 90) {
      return 'fa-battery-full';
    } else if (battery >= 65) {
      return 'fa-battery-three-quarters';
    } else if (battery >= 40) {
      return 'fa-battery-half';
    } else if (battery >= 15) {
      return 'fa-battery-quarter';
    } else {
      return 'fa-battery-empty';
    }
  }

  $: batteryIcon = getBatteryIcon(batteryChargingManual, battery);

  function showFmsStatus() {
    if (!fmsStatus) {
      return;
    }
    modalStore.trigger(alertModal('FMS Status', fmsStatus));
  }

  async function stopRobot() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping AGV controller ?',
      position: 'items-center',
      response: async (r) => {
        if (!r) {
          return;
        }
        const spinner = {
          type: 'component',
          component: 'modalLoadingSpinner',
          meta: { content: 'Stopping AGV Controller' }
        };
        modalStore.trigger(spinner);
        try {
          await agv05Boot.stopRobot();
        } catch (error) {
          console.error('Fail to stop robot.');
        }
        modalStore.close(spinner);
      }
    };
    modalStore.trigger(modal);
  }
</script>

<AppBar
  gridColumns="grid-cols-[auto_1fr_auto]"
  gap="gap-1 md:gap-4"
  slotDefault="place-self-center"
  slotTrail="place-content-end"
  background="shadow-lg bg-primary-600-300-token">
  <svelte:fragment slot="lead">
    <a href="/" class="flex flex-row">
      <img src={logo} class="h-8 w-16 max-w-none self-center" alt="Logo" />
      <div class="hidden flex-col text-white md:flex">
        <span class="text-sm font-bold lg:text-lg">
          {$t('user_panel.title')}
        </span>
        <span class="text-xs md:max-w-[160px] lg:text-sm">{$robotName}&nbsp;</span>
      </div>
    </a>
  </svelte:fragment>
  <svelte:fragment slot="trail">
    {#if $popupType === 'safety'}
      <button
        type="button"
        class="variant-filled-error btn px-2.5"
        class:bounce={$safetyNotification}
        on:click={() => {
          mainController.popupService.retrieve();
        }}>
        <i class="fa-solid fa-triangle-exclamation m-1"></i>
      </button>
    {/if}
    {#if $hasTracker}
      <button
        type="button"
        class="variant-filled-error btn px-2.5"
        on:click={mainController.downtimeTrackerService.showTracker}>
        <i class="fa-solid fa-bug m-1"></i>
      </button>
    {/if}
    {#if isFmsMode}
      <button type="button" class="variant-filled-error btn p-1.5" on:click={showFmsStatus}>
        <i class="fa-solid m-1 {fmsBroken ? 'fa-chain-broken' : 'fa-link'}"></i>
      </button>
    {/if}
    {#if $popupType === 'user'}
      <button
        type="button"
        class="variant-filled-primary bg-primary-400-500-token btn px-2.5 text-black"
        on:click={() => {
          mainController.popupService.retrieve();
        }}>
        <i class="fa-solid fa-circle-info m-1"></i>
      </button>
    {/if}
    {#if diskSpaceWarning}
      <button
        type="button"
        class="variant-filled-error btn px-2.5"
        on:click={showDiskSpaceWarning}>
        <i class="fa-solid fa-hard-drive m-1"></i>
      </button>
    {/if}
    {#if maintenanceWarning}
      <button
        type="button"
        class="variant-filled-error btn px-2.5"
        on:click={showMaintenanceWarning}>
        <i class="fa-solid fa-screwdriver-wrench m-1"></i>
      </button>
    {/if}
    <div class="flex flex-col">
      <WifiButton />
    </div>
    {#if $robotRunning && battery >= 0 && battery <= 100}
      <div
        class="variant-filled-primary bg-primary-400-500-token flex h-[42px] select-none items-center justify-between rounded-full px-4 text-black">
        <div class="text-[6px]">
          <span class="fa-stack fa-2x">
            <i class="fa-solid fa-stack-2x {batteryIcon} mr-0 lg:mr-2"></i>
            {#if batteryCharging}
              <i class="fa-solid fa-stack-1x fa-bolt text-yellow-500"></i>
            {/if}
          </span>
        </div>
        <div class="ml-2">{battery}%</div>
      </div>
    {/if}
    {#if !$robotRunning && countdown !== null}
      <button
        type="button"
        class="variant-filled-error btn h-[42px] bg-error-600 px-1 text-white md:px-5"
        on:click={extendCountdown}>
        <div
          class="line-h hidden text-right text-[0.55rem] leading-none tracking-widest md:block">
          Auto<br />Power off
        </div>
        <span class="!ml-0 w-9 text-sm md:!ml-2 md:text-lg">
          {countdown}
        </span>
      </button>
    {/if}
    {#if $robotRunning}
      <button type="button" class="variant-filled-error btn px-2.5" on:click={stopRobot}>
        <i class="fa-solid fa-power-off m-1"></i>
      </button>
    {/if}
    <TimeDisplay />
    <UserMenu class="variant-filled-primary bg-primary-400-500-token text-black">
      <Perm perms="system.view_system_panel">
        <li>
          <a href="/config" class="btn w-full">{$t('common.config_panel')}</a>
        </li>
      </Perm>
    </UserMenu>
  </svelte:fragment>
</AppBar>

<style>
.bounce { animation: bounce 0.5s ease-in-out infinite; }
@keyframes bounce {
  0%, 100% { transform: translateY(0) }
  50%      { transform: translateY(-6px) }
}
</style>
