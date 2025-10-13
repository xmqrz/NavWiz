<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { onDestroy } from 'svelte';

  import { agv05Boot } from '$lib/shared/services/user-panel/agv05-boot.service.js';
  import { httpErrorModal } from '$lib/modal-service.js';
  import { t } from '$lib/translations';
  import { getEnv } from 'stores/auth';
  import logo from '$lib/assets/img/navwiz_logo.svg';
  import UserLayout from 'components/UserLayout.svelte';

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let hasAlternateStart = getEnv('TRACKLESS') ? true : false;
  let softRebootTimeout;
  let softRebootSpinner;

  function startRobot(mode, isRetry = false) {
    const spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: { content: 'Starting AGV Controller' }
    };

    modalStore.trigger(spinner);
    agv05Boot
      .startRobot(mode)
      .then(function (data) {
        modalStore.close(spinner);
        if (data.cancelled) {
          return;
        }
        if (data.result === true) {
          toastStore.trigger({
            message: 'AGV controller started.',
            timeout: 3000,
            hoverable: true
          });
        } else {
          modalStore.trigger({
            type: 'alert',
            title: 'Failed to start AGV controller.',
            body: data.message.replace(/\n/g, '<br/>'),
            buttonTextCancel: 'Ok'
          });
        }
      })
      .catch(async function (e) {
        modalStore.close(spinner);
        if (!isRetry && e.errorCode === 502) {
          startRobot(mode, true);
        } else {
          let msg = e.message;
          if (e.cause) {
            try {
              const d = await e.cause.json();
              if (d.detail) {
                msg = d.detail;
              }
            } catch {
              console.log('fail to parse error reason in startRobot');
            }
          }
          modalStore.trigger({
            type: 'alert',
            title: 'Failed to start AGV controller.',
            body: msg
          });
        }
      });
  }

  function showStartMenu() {
    const modal = {
      type: 'component',
      component: 'modalAlternateStartMenu',
      response: (r) => {
        if (!r) {
          return;
        }
        startRobot(r);
      }
    };
    modalStore.trigger(modal);
  }

  function softReboot() {
    const modal = {
      type: 'confirm',
      title: 'Perform soft reboot?',
      position: 'items-center',
      response: (r) => {
        if (r) {
          if (softRebootSpinner) {
            modalStore.close(softRebootSpinner);
          }
          softRebootSpinner = {
            type: 'component',
            component: 'modalLoadingSpinner',
            meta: { content: 'Soft rebooting now' }
          };
          modalStore.trigger(softRebootSpinner);
          softRebootTimeout = setTimeout(function () {
            modalStore.close(softRebootSpinner);
            window.location.reload();
          }, 20000);

          agv05Boot
            .softReboot()
            .then((data) => {
              if (data.cancelled) {
                clearTimeout(softRebootTimeout);
                modalStore.close(softRebootSpinner);
                return;
              }
            })
            .catch(function (http) {
              clearTimeout(softRebootTimeout);
              modalStore.close(softRebootSpinner);
              modalStore.trigger(httpErrorModal('Failed to perform soft reboot', http));
            });
        }
      }
    };
    modalStore.trigger(modal);
  }

  function powerOff() {
    const modal = {
      type: 'confirm',
      title: 'Power off AGV?',
      position: 'items-center',
      response: (r) => {
        if (r) {
          const spinner = {
            type: 'component',
            component: 'modalLoadingSpinner',
            meta: { content: 'Powering off now' }
          };
          modalStore.trigger(spinner);

          agv05Boot
            .powerOff()
            .then((data) => {
              if (data.cancelled) {
                modalStore.close(spinner);
                return;
              }
            })
            .catch(function (http) {
              modalStore.close(spinner);
              modalStore.trigger(httpErrorModal('Failed to power off.', http));
            });
        }
      }
    };
    modalStore.trigger(modal);
  }

  onDestroy(() => {
    clearTimeout(softRebootTimeout);
    if (softRebootSpinner) {
      modalStore.close(softRebootSpinner);
    }
  });
</script>

<UserLayout mode="0">
  <div class="flex w-full flex-col">
    <img src={logo} class="mt-16 w-4/12 self-center shadow-xl md:w-2/12" alt="NavWiz Logo" />

    <div class="mb-4 mt-12 flex w-full max-w-sm flex-col self-center lg:mt-24 lg:max-w-xl">
      <div
        class="variant-filled-success btn-group rounded-none rounded-t-3xl [&>*+*]:border-none">
        <button
          type="button"
          class="h-16 w-full !justify-start font-bold"
          on:click={() => startRobot(1)}>
          {$t('user_panel.start_btn')}
        </button>
        {#if hasAlternateStart}
          <button type="button" on:click={showStartMenu}>
            <i class="fa-solid fa-chevron-down"></i>
          </button>
        {/if}
      </div>
      <button
        type="button"
        class="variant-filled-warning btn h-16 justify-between rounded-none font-bold"
        on:click={softReboot}>{$t('user_panel.reboot_btn')}</button>
      <button
        type="button"
        class="variant-filled-error btn h-16 justify-between rounded-none rounded-b-3xl font-bold shadow-xl"
        on:click={powerOff}>{$t('user_panel.poweroff_btn')}</button>
    </div>
  </div>
</UserLayout>
