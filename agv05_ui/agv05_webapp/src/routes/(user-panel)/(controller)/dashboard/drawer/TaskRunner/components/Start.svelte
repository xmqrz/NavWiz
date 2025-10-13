<script>
  export let fmsBroken;
  export let fmsStatus;
  export let defaultModule;

  export let moduleManager;
  export let taskRunner;

  let autoStart = false;

  $: autoStart = defaultModule && defaultModule.module === 'task-runner';

  function cancelDefaultModule() {
    moduleManager.publish({
      command: 'cancel_default_module'
    });
  }

  function startModule() {
    taskRunner.startModule();
  }
</script>

{#if fmsBroken}
  <div class="p-3 text-lg text-red-600">
    <!-- eslint-disable-next-line svelte/no-at-html-tags -->
    DFleet error: <span>{@html fmsStatus}</span>
  </div>
{/if}
{#if defaultModule && !fmsBroken}
  <div>
    <button
      type="button"
      class="self-right btn-sm"
      tip-title="Stop auto-start app"
      on:click={cancelDefaultModule}>
      <i class="fa-solid fa-close"></i>
      Starting "{defaultModule.module}" app in {defaultModule.countdown} second(s)...
    </button>
  </div>
{/if}
<button
  type="button"
  class:variant-filled-error={autoStart}
  class:variant-filled-surface={!autoStart}
  class="btn btn-lg max-w-2xl overflow-hidden"
  on:click={startModule}>
  <i class="fa-solid fa-play mr-2"></i>
  <span class="truncate">Start Task Runner</span>
</button>
