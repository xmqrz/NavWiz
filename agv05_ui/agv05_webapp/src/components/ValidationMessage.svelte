<script>
  import { onMount } from 'svelte';
  import { getModalStore, popup } from '@skeletonlabs/skeleton';

  import { alertModal, httpErrorModal } from '$lib/modal-service.js';
  import { validationData } from 'stores/validation-data';
  import { resourceIdToUrl } from '$lib/utils';
  import { agv05Boot } from '$lib/shared/services/user-panel/agv05-boot.service.js';

  onMount(() => {
    const unsubscribe = validationData.subscribe(callback);

    return () => {
      unsubscribe();
    };
  });

  const modalStore = getModalStore();

  let message = '';
  let visible = false;
  let error = false;
  let validating_visible = false;
  let softReboot = false;
  let hotReload = false;
  let timeout, spinner;

  function callback(data) {
    if (data.id !== 'validation-data') {
      return;
    }
    if (data.state === 'clean') {
      visible = false;
      validating_visible = false;
    } else if (data.state === 'valid') {
      visible = true;
      validating_visible = false;
      error = false;
      message = data.msg;
      softReboot = true;
      hotReload = false;
    } else if (data.state === 'reloadable') {
      visible = true;
      validating_visible = false;
      error = false;
      message = data.msg;
      softReboot = true;
      hotReload = true;
    } else if (data.state === 'invalid') {
      visible = true;
      validating_visible = false;
      error = true;
      message = renderErrorMessage(data);
      softReboot = false;
      hotReload = false;
    } else if (data.state === 'validating') {
      visible = false;
      validating_visible = true;
      error = false;
      message = data.msg;
    }
  }

  function renderErrorMessage(data) {
    let msg = data.msg;
    let params = data.params;
    for (let key in params) {
      let val = params[key];
      msg = msg.replace(
        `{${key}}`,
        `<a class="anchor text-gray-50 font-bold" href="${resourceIdToUrl(val.id, val.item)}">${val.name}</a>`
      );
    }
    return `Invalid changes detected: ${msg}`;
  }

  function cancelSpinner() {
    if (spinner) {
      modalStore.close(spinner);
      spinner = undefined;
    }
    if (timeout) {
      clearTimeout(timeout);
      timeout = undefined;
    }
  }

  function onHotReload() {
    const modal = {
      type: 'confirm',
      title: 'Perform hot reload?',
      position: 'items-center',
      response: (r) => {
        if (!r) {
          return;
        }
        cancelSpinner();
        spinner = {
          type: 'component',
          component: 'modalLoadingSpinner',
          meta: { content: 'Hot reloading now' }
        };
        modalStore.trigger(spinner);
        timeout = setTimeout(function () {
          modalStore.close(spinner);
        }, 10000);
        agv05Boot
          .hotReload()
          .then(function (data) {
            if (data.cancelled) {
              cancelSpinner();
              return;
            }
            if (data.result !== true) {
              modalStore.trigger(
                alertModal('Failed to hot reload', data.message.replace(/\n/g, '<br/>'))
              );
              cancelSpinner();
              return;
            }
          })
          .catch(function (http) {
            modalStore.trigger(httpErrorModal('Failed to hot reload.', http));
            cancelSpinner();
          });
      }
    };
    modalStore.trigger(modal);
  }

  function onSafeSoftReboot() {
    const modal = {
      type: 'confirm',
      title: 'Perform soft reboot?',
      position: 'items-center',
      response: (r) => {
        if (!r) {
          return;
        }
        cancelSpinner();
        spinner = {
          type: 'component',
          component: 'modalLoadingSpinner',
          meta: { content: 'Soft rebooting now' }
        };
        modalStore.trigger(spinner);
        timeout = setTimeout(function () {
          modalStore.close(spinner);
        }, 20000);
        agv05Boot
          .safeSoftReboot()
          .then(function (data) {
            if (data.cancelled) {
              cancelSpinner();
              return;
            }
            if (data.result !== true) {
              modalStore.trigger(
                alertModal('Failed to soft reboot', data.message.replace(/\n/g, '<br/>'))
              );
              cancelSpinner();
              return;
            }
          })
          .catch(function (http) {
            cancelSpinner();
            modalStore.trigger(httpErrorModal('Failed to soft reboot.', http));
          });
      }
    };
    modalStore.trigger(modal);
  }
</script>

{#if visible}
  <div class="pb-5">
    <aside class="alert {error ? 'variant-filled-error' : 'variant-filled-warning'}">
      <div>
        <span>
          <!-- eslint-disable-next-line svelte/no-at-html-tags -->
          {@html message}
        </span>
        {#if hotReload}
          <div class="inline-flex">
            <button
              type="button"
              class="variant-filled-warning btn btn-sm rounded-r-none bg-warning-300"
              on:click={onHotReload}>Hot Reload</button>
            <button
              type="button"
              class="variant-filled-warning btn btn-sm h-8 rounded-l-none bg-warning-300"
              use:popup={{
                event: 'click',
                target: 'soft-reboot-popup',
                placement: 'bottom'
              }}>
              <i class="fa-solid fa-caret-down"></i>
            </button>
          </div>
          <div class="z-50" data-popup="soft-reboot-popup">
            <ul class="rounded bg-warning-300">
              <li>
                <button type="button" class="btn btn-sm w-full" on:click={onSafeSoftReboot}>
                  Soft Reboot
                </button>
              </li>
            </ul>
          </div>
        {:else if softReboot}
          <button
            type="button"
            class="variant-filled-warning btn btn-sm bg-warning-300"
            on:click={onSafeSoftReboot}>Soft Reboot</button>
        {/if}
      </div>
    </aside>
  </div>
{/if}

{#if validating_visible}
  <div class="pb-5">
    <aside class="alert variant-filled-tertiary">
      <div class="alert-message">
        <p>
          <!-- eslint-disable-next-line svelte/no-at-html-tags -->
          {@html message}
          <i class="fa-solid fa-rotate fa-spin"></i>
        </p>
      </div>
    </aside>
  </div>
{/if}
