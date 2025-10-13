<script>
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import { getModalStore } from '@skeletonlabs/skeleton';

  export let taskTemplates;
  export let fmsBroken;
  export let fmsStatus;
  export let defaultInit;
  export let customInitOptions;
  export let initStatus;
  export let stationInit;
  export let preInitOptions;
  export let paused;
  export let resuming;

  export let taskRunner;
  export let getTaskParams;
  export let setPaused;

  let expandInitStation = false;
  let expandPreInit = false;

  const modalStore = getModalStore();

  function cancelDefaultInit() {
    taskRunner.publish({
      command: 'cancel_default_init'
    });
  }

  function cancelInit() {
    taskRunner.stopModule();
  }

  function initWith(option, station) {
    let prefix = 'custom_init_';
    if (option === 'station') {
      const modal = {
        type: 'confirm',
        title: `AGV parked at<br/>"${station === -1 ? 'Previous Task Runner Stopped' : station}" ?`,
        position: 'items-center',
        response: (r) => {
          if (r) {
            taskRunner.publish({
              command: 'location_hint',
              hint: 'station',
              station: station
            });
          }
        }
      };
      modalStore.trigger(modal);
    } else if (option.type.indexOf(prefix) === 0) {
      getTaskParams(option, `Initialize with "${option.label}"`).then(function (res) {
        if (res) {
          taskRunner.publish({
            command: 'location_hint',
            hint: option.type,
            params: res
          });
        }
      });
    } else {
      taskRunner.publish({
        command: 'location_hint',
        hint: option.type
      });
    }
  }

  function preInitWith(option) {
    let prefix = 'pre_init_';
    if (option.type.indexOf(prefix) === 0) {
      getTaskParams(option, `Run "${option.label}"`).then(function (res) {
        if (res) {
          taskRunner.publish({
            command: 'pre_init',
            type: option.type,
            params: res
          });
        }
      });
    }
  }
</script>

{#if fmsBroken}
  <div class="p-3 text-lg text-red-600">
    <!-- eslint-disable-next-line svelte/no-at-html-tags -->
    DFleet error: <span>{@html fmsStatus}</span>
  </div>
{/if}
<div>
  <strong>Choose an option below to initialize AGV location.</strong>
</div>
{#if !taskTemplates}
  <ProgressRadial width="w-10" />
{/if}
{#if defaultInit && !fmsBroken}
  <div>
    <button
      type="button"
      class="self-right btn-sm"
      tip-title="Stop auto-initialization"
      on:click={cancelDefaultInit}>
      <i class="fa-solid fa-close"></i>
      Auto-initializing in {defaultInit.countdown} second(s)...
    </button>
  </div>
{/if}
<div class="space-y-2">
  {#each customInitOptions as op}
    <div>
      <button
        type="button"
        class="variant-filled-surface btn btn-xl w-full max-w-2xl"
        disabled={initStatus !== 'unaware' || fmsBroken}
        class:!variant-filled-error={defaultInit && defaultInit.type === op.type}
        class:!variant-filled={initStatus === op.type}
        on:click={() => initWith(op)}>
        <span class="truncate">
          {op.label}
        </span>
      </button>
    </div>
  {/each}
</div>

{#if stationInit}
  <!-- TODO: change to accordion -->
  {@const expand = expandInitStation && initStatus === 'unaware' && !fmsBroken}
  <div class="space-y-2">
    <button
      type="button"
      class="variant-filled-surface btn btn-xl relative w-full max-w-2xl"
      on:click={() => (expandInitStation = !expandInitStation)}
      disabled={initStatus !== 'unaware' || fmsBroken}>
      <span class="truncate pr-2">AGV is parked at a station.</span>
      <i
        class:fa-caret-up={expand}
        class:fa-caret-down={!expand}
        class="fa-solid absolute right-7"></i>
    </button>
    {#if expand}
      <div>
        {#if stationInit && !stationInit.length}
          <div class="variant-filled-error">
            <div>No stations defined.</div>
          </div>
        {:else}
          {#if stationInit.at(-1) === -1}
            <button
              type="button"
              class="variant-filled-surface btn btn-xl relative w-full max-w-2xl"
              on:click={() => initWith('station', -1)}>
              <span class="truncate pr-2">AGV is parked at previous Task Runner stopped.</span>
            </button>
          {/if}
          {#each stationInit as _, stIndex}
            {#if stIndex % 3 === 0}
              <div class="my-3 grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
                {#each [0, 1, 2] as i}
                  {@const st = stationInit[stIndex + i]}
                  {#if st && st !== -1}
                    <button
                      type="button"
                      class="variant-filled-surface btn btn-lg overflow-hidden bg-surface-300"
                      on:click={() => initWith('station', st)}>
                      <span class="truncate">
                        {st}
                      </span>
                    </button>
                  {/if}
                {/each}
              </div>
            {/if}
          {/each}
        {/if}
      </div>
    {/if}
  </div>
{/if}
{#if preInitOptions.length}
  {@const expand = expandPreInit && initStatus === 'unaware'}
  <div>
    <button
      type="button"
      class="variant-filled-surface btn btn-xl relative w-full max-w-2xl"
      class:bg-slate-50={initStatus === 'pre_init'}
      on:click={() => (expandPreInit = !expandPreInit)}
      disabled={initStatus !== 'unaware'}>
      <span class="truncate pr-2">Pre init</span>
      <i
        class:fa-caret-up={expand}
        class:fa-caret-down={!expand}
        class="fa-solid absolute right-7"></i>
    </button>
    {#if expand}
      {#each preInitOptions as _, opIndex}
        {#if opIndex % 3 === 0}
          <div class="my-3 grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
            {#each [0, 1, 2] as i}
              {@const op = preInitOptions[opIndex + i]}
              {#if op}
                <button
                  type="button"
                  class="variant-filled-surface btn btn-lg w-full overflow-hidden bg-surface-300"
                  on:click={() => preInitWith(op)}>
                  <span class="truncate">
                    {op.label}
                  </span>
                </button>
              {/if}
            {/each}
          </div>
        {/if}
      {/each}
    {/if}
  </div>
{/if}
{#if paused !== null}
  <div>
    <button
      type="button"
      class="btn btn-sm {paused && !resuming
        ? 'variant-filled-warning'
        : 'variant-filled-surface'}"
      on:click={() => setPaused(!paused || resuming)}>
      <span class="mr-1 truncate">Start task runner in paused state</span>
      <i class="fa-regular {paused && !resuming ? 'fa-circle-check' : 'fa-circle'}"></i>
    </button>
  </div>
{/if}
<div>
  <button type="button" class="variant-filled-error btn btn-sm" on:click={cancelInit}>
    <i class="fa-solid fa-exclamation mr-2"></i>
    <span class="truncate">Cancel</span>
    {#if initStatus !== 'unaware'}
      <ProgressRadial width="ml-3 w-4" meter="stroke-surface-50 dark:stroke-surface-900" />
    {/if}
  </button>
</div>
