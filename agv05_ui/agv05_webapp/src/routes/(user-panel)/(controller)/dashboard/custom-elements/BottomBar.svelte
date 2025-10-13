<script>
  import * as _ from 'lodash-es';
  import { onMount } from 'svelte';
  import { AppBar } from '@skeletonlabs/skeleton';

  import { allBools } from '$lib/utils';
  import { fmsChannel } from 'stores/sock/index.js';

  export let customElementAPI;

  const moduleManager = customElementAPI.moduleManager;
  const taskRunner = customElementAPI.taskRunner;

  let ready = false;
  let activeModule;
  let currentAction = '';
  let paused = null;
  let resuming = false;
  let batteryLow = false;
  let status = '-';
  let tasks = [];
  let initialized = false;
  let fmsStatus = null;
  let task;
  let fmsBroken = false;

  $: isFmsMode = !!fmsStatus;

  function updateStatus(activeModule, paused, resuming, batteryLow, tasks, initialized) {
    if (activeModule !== 'task-runner') {
      if (activeModule) {
        status = 'Running App';
      } else {
        status = 'Not Ready';
      }
      return;
    }

    if (!initialized) {
      status = 'Not Ready';
      return;
    }

    status = 'Idle';
    task = undefined;

    for (const t of tasks) {
      if (
        (!isFmsMode || t.agv_id === true) &&
        ['In_progress', 'Aborting'].indexOf(t.status) >= 0
      ) {
        task = t;
      }
    }

    if (task) {
      status = 'Working';
      if (paused) {
        status = 'Paused';
      }
    } else if (paused) {
      status = 'Suspended';
    }
  }
  $: updateStatus(activeModule, paused, resuming, batteryLow, tasks, initialized);

  function openTaskRunner() {
    customElementAPI.openTaskRunner();
  }

  function openTaskHistory() {
    customElementAPI.openTaskHistory();
  }

  function openTaskHistoryRight() {
    customElementAPI.openTaskHistory('right');
  }

  function moduleManagerCallback(data) {
    if (data.command === 'active_module') {
      ready = true;
      activeModule = data.module_id;
      if (data.module_id === 'task-runner') {
        taskRunnerStarted();
      }
    }
  }

  function taskRunnerCallback(data) {
    if (data.command === 'action') {
      currentAction = data.action || '';
    } else if (data.command === 'list_tasks') {
      if (Array.isArray(data.tasks)) {
        tasks = data.tasks;
      }
    } else if (data.command === 'init_status') {
      initialized = data.status === 'initialized';
    } else if (data.command === 'is_paused') {
      paused = data.paused;
      resuming = data.resuming;
      batteryLow = data.battery_low;
    }
  }

  function fmsCallback(data) {
    if (data.id === 'status') {
      let status = data.value.replace(/\n/g, ' ');
      let broken = data.broken;
      if (fmsStatus !== status || fmsBroken !== broken) {
        fmsStatus = status;
        fmsBroken = broken;
      }
    }
  }

  function taskRunnerStarted() {
    batteryLow = false;
    fmsBroken = false;
    fmsStatus = null;
    initialized = false;
    paused = null;
    resuming = false;
    taskRunner.publish({
      command: 'init_status'
    });
    taskRunner.publish({
      command: 'list_tasks'
    });
    taskRunner.publish({
      command: 'is_paused'
    });
  }

  onMount(() => {
    moduleManager.subscribe(moduleManagerCallback);
    taskRunner.subscribe(taskRunnerCallback);
    fmsChannel.subscribe(fmsCallback);
    setTimeout(() => {
      moduleManager.publish({
        command: 'active_module'
      });
    }, 300);

    return () => {
      fmsChannel.unsubscribe(fmsCallback);
      taskRunner.unsubscribe(taskRunnerCallback);
      moduleManager.unsubscribe(moduleManagerCallback);
    };
  });
</script>

<div class="float-right-btn">
  <button
    type="button"
    on:click={openTaskHistoryRight}
    class="variant-filled-surface btn rounded-r-none bg-surface-800 py-3 pl-3 pr-2">
    <i class="fa-solid fa-chevron-left fa-xs text-gray-400"></i>
    <i class="fa-solid fa-history text-gray-200"></i>
  </button>
</div>
<div class="fixed bottom-4 left-24 right-4 z-[1]">
  <AppBar
    border="rounded-2xl"
    padding="p-[6px]"
    gridColumns="grid-cols-1fr sm:grid-cols-[1fr_auto]"
    slotDefault="place-content-start min-w-0"
    slotTrail="h-full hidden sm:block"
    background="shadow-xl bg-surface-800-100-token">
    <svelte:fragment slot="default">
      <button
        on:click={openTaskRunner}
        type="button"
        class="bg-surface-200-700-token absolute left-[40%] top-[2px] h-[2px] w-[20%] rounded-full">
      </button>
      <button type="button" class="relative flex w-full space-x-3" on:click={openTaskRunner}>
        <div class="flex flex-grow truncate text-left lg:flex-grow-0">
          <div
            class="bg-surface-400-500-token border-surface-600-300-token flex min-w-[140px] select-none flex-col items-start rounded-l-xl border-r-2 py-2 pl-2 pr-4 text-black">
            <div class="flex space-x-2">
              <span class="text-xs text-gray-700">Task Runner</span>
              {#if batteryLow}
                <span class="fa-stack fa-xs h-4 w-4">
                  <i class="fa-solid fa-stack-1x fa-battery-quarter -top-1 text-red-500"></i>
                  <i class="fa-solid fa-stack-1x fa-battery-empty -top-1"></i>
                </span>
              {/if}
              {#if paused || resuming}
                <span
                  class="chip h-4"
                  class:variant-filled-warning={paused && !resuming}
                  class:variant-filled-success={resuming}>
                  {#if resuming}
                    <i class="fa-solid fa-play fa-fade text-xs"></i>
                  {:else}
                    <i class="fa-solid fa-pause text-xs"></i>
                  {/if}
                </span>
              {/if}
            </div>
            {#if !ready}
              <span class="placeholder my-1 h-3 w-12 animate-pulse" />
            {:else}
              <div class="text-sm">
                <span>{status}</span>
              </div>
            {/if}
          </div>
          <div
            class="bg-surface-400-500-token flex flex-grow select-none flex-col items-start rounded-r-xl py-2 pl-4 pr-6 text-black">
            <span class="text-xs text-gray-700">
              {task ? task.name : 'No Task'}{#if task && task.progress}
                &nbsp;| <b>{task.progress}</b>{/if}
            </span>
            <span class="text-sm">
              {#if task && task.params.length}
                {#each task.params as param}
                  {#if !allBools(task.params) || param.value}
                    <span>
                      | <small><strong>{_.startCase(param.name)}</strong></small>
                      {#if !allBools(task.params)}
                        <span class="pr-1">: {param.value}</span>
                      {/if}
                    </span>
                  {/if}
                {/each}
              {:else}
                -
              {/if}
            </span>
          </div>
        </div>
        {#if fmsBroken}
          <div
            class="hidden select-none flex-col justify-around text-wrap rounded-xl bg-red-500 px-4 text-sm text-black lg:flex">
            <!-- eslint-disable-next-line svelte/no-at-html-tags -->
            DFleet error: {@html fmsStatus}
          </div>
        {:else}
          <div
            class="hidden select-none flex-col justify-around text-wrap px-4 text-sm text-white lg:flex">
            {currentAction}
          </div>
        {/if}
      </button>
    </svelte:fragment>
    <svelte:fragment slot="trail">
      <button
        on:click={openTaskHistory}
        type="button"
        class="variant-filled-surface btn h-full rounded-xl bg-surface-800 text-gray-200">
        <i class="fa-solid fa-history"></i>
      </button>
    </svelte:fragment>
  </AppBar>
</div>

<style lang="postcss">
  .float-right-btn {
    @apply fixed;
    @apply right-0;
    @apply flex flex-col space-y-3 sm:hidden;
  }
</style>
