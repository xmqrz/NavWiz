<script>
  import * as _ from 'lodash-es';
  import { getContext } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import fadeScale from '$lib/transitions/fade-scale.js';

  import { getEnv } from 'stores/auth.js';

  export let isFmsMode;
  export let fmsBroken;
  export let fmsStatus;
  export let paused;
  export let batteryLow;
  export let resuming;

  export let taskRunner;
  export let showIo;
  export let showRemoteIo;
  export let showTaskTemplate;
  export let showSetPose;
  export let setPaused;

  const modalStore = getModalStore();
  const user = getContext('user');

  function stopModule() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module <br/> "Task Runner" ?',
      position: 'items-center',
      response: (r) => {
        if (r) {
          taskRunner.stopModule();
        }
      }
    };
    modalStore.trigger(modal);
  }
</script>

{#if isFmsMode}
  <div>
    {#if !fmsBroken}
      <div class="p-3 text-lg">
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        DFleet mode: <span>{@html fmsStatus}</span>
      </div>
    {:else}
      <div class="p-3 text-lg text-red-600">
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        DFleet error: <span>{@html fmsStatus}</span>
      </div>
    {/if}
  </div>
{/if}
{#if !paused && batteryLow}
  <div transition:fadeScale={{ duration: 200 }}>
    <h3 class="variant-filled-error rounded-2xl py-10 text-center text-black">Battery Low!</h3>
  </div>
{/if}
{#if paused}
  <div id="pause-status">
    <div transition:fadeScale={{ duration: 200 }}>
      {#if !resuming}
        <h3 class="variant-filled-warning rounded-2xl py-10 text-center text-black">
          Task runner is paused!
        </h3>
      {:else}
        <h3 class="variant-filled-success rounded-2xl py-10 text-center text-black">
          Task runner is resuming!
        </h3>
      {/if}
    </div>
  </div>
{/if}
<div>
  <div>
    <div class="float-left space-x-2">
      {#if $user.has_perms('app.show_panel_live_monitor')}
        <button
          type="button"
          class="variant-filled-secondary btn btn-sm py-2 text-2xl"
          on:click={showIo}
          tip-title="IO Live View">
          <i class="fa-solid fa-dice-six fa-fw"></i>
          <!-- <div class="font-mono tracking-tighter leading-none font-semibold">I/O</div> -->
        </button>
        <button
          type="button"
          class="variant-filled-secondary btn btn-sm py-2 text-2xl"
          on:click={showRemoteIo}
          tip-title="Remote IO Live View"><i class="fa-solid fa-calculator fa-fw"></i></button>
        <button
          type="button"
          class="variant-filled-secondary btn btn-sm py-2 text-2xl"
          on:click={showTaskTemplate}
          tip-title="Action Live View">
          <i class="fa-solid fa-code-merge fa-fw"></i>
        </button>
        {#if getEnv('TRACKLESS') && $user.has_perms('app.show_panel_live_manipulation')}
          <button
            type="button"
            class="variant-filled-secondary btn btn-sm py-2 text-2xl"
            on:click={showSetPose}
            tip-title="Set Pose">
            <i class="fa-solid fa-location-crosshairs fa-fw"></i>
          </button>
        {/if}
      {/if}
    </div>
    <div class="float-right space-x-2">
      {#if !paused && $user.has_perms('app.pause_task_runner')}
        <button
          type="button"
          class="variant-filled-warning btn btn-sm"
          on:click={() => setPaused(true)}>
          <i class="fa-regular fa-circle-pause fa-2x fa-fw mr-2"></i>Pause
        </button>
      {/if}
      {#if paused && !resuming && $user.has_perms('app.resume_task_runner')}
        <button
          type="button"
          class="variant-filled-success btn btn-sm"
          on:click={() => setPaused(false)}>
          <i class="fa-regular fa-circle-play fa-2x mr-2"></i>Resume
        </button>
      {/if}
      {#if paused && resuming && $user.has_perms('app.resume_task_runner')}
        <button type="button" class="variant-filled-success btn btn-sm" disabled>
          <i class="fa-regular fa-circle-play fa-2x mr-2"></i>Resuming...
        </button>
      {/if}
      <button
        type="button"
        class="icon-left ion-alert variant-filled-error btn btn-sm"
        on:click={stopModule}>
        <i class="fa-solid fa-circle-stop fa-2x mr-2"></i>Stop Task Runner</button>
    </div>
  </div>
</div>
