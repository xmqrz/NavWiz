<script>
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import { createEventDispatcher, getContext, onMount } from 'svelte';
  import { goto } from '$app/navigation';

  import KeyboardSpacer from 'components/keyboard/KeyboardSpacer.svelte';

  export let moduleID;
  export let title = '(missing title)';
  export let keyboardSpacer = true;

  const dispatch = createEventDispatcher();

  const activeModule = getContext('activeModule');

  function onClose() {
    dispatch('close');
  }

  function onMinimize() {
    goto('/apps', { replaceState: true });
  }

  function onActiveModule(am) {
    if (am !== '-' && am !== moduleID) {
      onMinimize();
    }
  }

  onMount(() => {
    // NOTE: subscribe being handled first before template redraw compare to reactive statement.
    const unsubscribe = activeModule.subscribe(onActiveModule);
    return () => {
      unsubscribe();
    };
  });
</script>

<div class="flex h-full flex-col">
  <div class="variant-filled-tertiary relative py-2 text-center">
    <span class="text-lg">{title}</span>
    <span class="absolute right-3">
      <button
        type="button"
        class="variant-ringed-surface btn btn-sm w-10 rounded ring-white"
        on:click={onMinimize}>
        <i class="fa fa-window-minimize"></i>
      </button>
      <button
        type="button"
        disabled={$activeModule === '-'}
        class="variant-ringed-surface btn btn-sm w-10 rounded ring-white"
        on:click={onClose}>
        <i class="fa fa-close"></i>
      </button>
    </span>
  </div>
  <div class="h-full w-full flex-1 overflow-auto overflow-y-scroll">
    {#if $activeModule === moduleID}
      <slot />
    {:else if $activeModule === '-'}
      <div class="flex justify-center">
        <ProgressRadial width="w-24 p-5" />
      </div>
    {/if}
  </div>
  {#if keyboardSpacer}
    <KeyboardSpacer />
  {/if}
</div>
