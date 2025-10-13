<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { countdownTimer } from 'stores/states.js';

  // Props
  // svelte-ignore unused-export-let
  export let parent;

  const modalStore = getModalStore();

  function response(v) {
    if ($modalStore[0].response) {
      $modalStore[0].response(v);
    }
    modalStore.close();
  }

  function onKeypad() {
    response(this.innerText);
  }

  function onTrue() {
    response(true);
  }

  function onFalse() {
    response(false);
  }

  function close() {
    response();
  }

  function mute() {
    response('mute');
  }
</script>

{#if $modalStore[0]}
  <div class="fixed inset-0 flex justify-center">
    <div class="card min-w-[400px] self-center p-3">
      <header class="card-header relative h-12 p-3 text-center text-xl font-bold">
        {$modalStore[0].title}
        <button type="button" class="btn-icon absolute right-0 top-0" on:click={close}>
          <i class="fa fa-chevron-up"></i>
        </button>
        {#if $modalStore[0].msgType?.startsWith('safety-')}
          <button type="button" class="btn-icon absolute right-[44px] top-0" on:mouseup={mute}>
            <i class="fa fa-bell-slash"></i>
          </button>
        {/if}
      </header>
      <hr />
      <section class="grid grid-cols-6 gap-3 p-3">
        {#if $modalStore[0].displayCountdown}
          <div class="col-span-6 text-center">{$countdownTimer}</div>
        {/if}
        {#if $modalStore[0].message}
          <!-- eslint-disable-next-line svelte/no-at-html-tags -->
          <div class="col-span-6">{@html $modalStore[0].message}</div>
        {/if}
        {#if $modalStore[0].msgType === 'non-interactive'}
          <!-- empty block -->
        {:else if $modalStore[0].msgType === 'alert'}
          <button
            type="button"
            class="variant-filled-success btn btn-xl col-span-6 text-center"
            on:click={onTrue}>OK</button>
        {:else if $modalStore[0].msgType === 'confirm'}
          <button
            type="button"
            class="variant-filled-success btn btn-xl col-span-3 text-center"
            on:click={onTrue}>Yes</button>
          <button
            type="button"
            class="variant-filled-error btn btn-xl col-span-3 text-center"
            on:click={onFalse}>No</button>
        {:else if $modalStore[0].msgType === 'keypad'}
          {#each ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', '0', 'B'] as n}
            <button
              type="button"
              class="variant-filled-surface btn btn-xl col-span-2 text-center"
              on:click={onKeypad}>{n}</button>
          {/each}
        {:else if $modalStore[0].msgType === 'safety-triggered'}
          <!-- empty block -->
        {:else if $modalStore[0].msgType === 'safety-resume'}
          <button
            type="button"
            class="variant-filled-success btn btn-xl col-span-6 text-center"
            on:click={onTrue}>Resume</button>
        {:else if $modalStore[0].msgType === 'safety-resuming'}
          <!-- empty block -->
        {/if}
      </section>
    </div>
  </div>
{/if}
