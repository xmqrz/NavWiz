<script>
  import { getModalStore, ProgressRadial } from '@skeletonlabs/skeleton';
  import { onDestroy, onMount, getContext } from 'svelte';
  import { goto } from '$app/navigation';

  import { alertModal } from '$lib/modal-service.js';

  import { fmsChannel } from 'stores/sock/index.js';
  import { getEnv } from 'stores/auth';

  const mainController = getContext('mainController');
  const moduleManager = mainController.moduleService.getManager();
  const modalStore = getModalStore();
  const activeModule = getContext('activeModule');
  let hwAppList;

  const modules = [
    {
      id: 'skill-test',
      name: 'Action Test'
    },
    {
      id: 'calibration',
      name: 'Calibration'
    },
    {
      id: 'dev-mode',
      name: 'Development Mode'
    },
    {
      id: 'hardware-test',
      name: 'Hardware Test'
    },
    {
      id: 'homing',
      name: 'Homing'
    },
    getEnv('TRACKLESS')
      ? {
          id: 'manual-control',
          name: 'Manual Control'
        }
      : undefined,
    {
      id: 'manual-line-follow',
      name: 'Manual Line Follow'
    },
    {
      id: 'wifi-test',
      name: 'Wifi Test'
    }
  ].filter(Boolean);

  let pendingOpen = null;
  let defaultModule = null;

  function cancelDefaultModule() {
    moduleManager.publish({
      command: 'cancel_default_module'
    });
  }

  function showModule(module) {
    goto(`/apps/${module.id}`, { replaceState: true });
  }

  function openModule(module) {
    pendingOpen = null;
    if (module.id === $activeModule) {
      const modal = {
        type: 'confirm',
        title: 'The module is currently active.',
        body: 'Do you want to resume or stop the module? Click on the backdrop to cancel.',
        buttonTextConfirm: 'Resume',
        buttonTextCancel: 'Stop',
        response: (r) => {
          if (r === true) {
            showModule(module);
          } else if (r === false) {
            moduleManager.stopModule(module.id);
          }
        }
      };
      modalStore.trigger(modal);
    } else {
      pendingOpen = module;
      moduleManager.startModule(module.id);
    }
  }

  function moduleManagerCallback(data) {
    if (data.command === 'start_module') {
      if (pendingOpen && pendingOpen.id === data.module_id) {
        if (data.success) {
          showModule(pendingOpen);
        } else {
          modalStore.trigger(
            alertModal(
              'Error',
              `Unable to start module "${pendingOpen.name}". Is there another module running now?`
            )
          );
        }
      }
      pendingOpen = null;
    } else if (data.command === 'stop_module') {
      pendingOpen = null;
    } else if (data.command === 'default_module') {
      defaultModule = data.default_module;
    }
  }

  /* fms status update */
  let fmsStatus = null;
  let fmsBroken = false;

  function fmsCallback(data) {
    if (data.id === 'status') {
      fmsStatus = data.value.replace(/\n/g, '<br/>');
      fmsBroken = data.broken;
    }
  }

  onMount(() => {
    // TODO: consider using store to save state between modules and task runner.
    moduleManager.subscribe(moduleManagerCallback);
    fmsChannel.subscribe(fmsCallback);
    // allow time for subscribe to happen first.
    setTimeout(function () {
      moduleManager.publish({
        command: 'active_module'
      });
    }, 100);

    moduleManager.getHwApp().then(function (data) {
      hwAppList = data;
    });
  });

  onDestroy(() => {
    // $log.debug($scope.controllerName, '$destroy.');
    fmsChannel.unsubscribe(fmsCallback);
    moduleManager.unsubscribe(moduleManagerCallback);
  });

  // $log.debug($scope.controllerName, 'initialized.');
</script>

<div>
  <nav class="list-nav">
    <div class="p-2 px-4">
      <span class="text-lg font-bold">Apps</span>
    </div>
    {#if fmsBroken}
      <div class="bg-slate-200">
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        DFleet error: <span>{@html fmsStatus}</span>
      </div>
    {/if}
    {#if defaultModule && !fmsBroken}
      <div class="bg-slate-200">
        <button
          type="button"
          class="variant-filled-error btn btn-sm"
          title="Stop auto-start app"
          on:click={cancelDefaultModule}>
          Starting "{defaultModule.module}" app in {defaultModule.countdown} second(s)...
          <i class="fa-solid fa-close"></i>
        </button>
      </div>
    {/if}
    <ul class="!space-y-0 text-2xl">
      {#each modules as m, i}
        <li class:border-t={i === 0} class="border-b border-surface-400">
          <button
            type="button"
            class="w-full !rounded-none !py-5 text-left"
            on:click={() => openModule(m)}>
            {#if m === pendingOpen}
              <ProgressRadial width="w-7" />
            {:else}
              <span class="badge h-7 w-7 bg-gray-500 text-white">
                <i class="fa-solid fa-arrow-right"></i>
              </span>
            {/if}
            <span>{m.name}</span>
            {#if m.id === $activeModule}
              <i class="fa-solid fa-fire text-red-600"></i>
            {/if}
          </button>
        </li>
      {/each}
      {#if hwAppList}
        {#each hwAppList as m}
          <li class="border-b border-surface-400">
            <button
              type="button"
              class="w-full !rounded-none !py-5 text-left"
              on:click={() => openModule(m)}>
              {#if m === pendingOpen}
                <ProgressRadial width="w-7" />
              {:else}
                <span class="badge h-7 w-7 bg-gray-500 text-white">
                  <i class="fa-solid fa-arrow-right"></i>
                </span>
              {/if}
              <span>{m.name}</span>
              {#if m.id === $activeModule}
                <i class="fa-solid fa-fire text-red-600"></i>
              {/if}
            </button>
          </li>
        {/each}
      {/if}
    </ul>
  </nav>
</div>
