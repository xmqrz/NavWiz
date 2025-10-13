<script>
  import { onMount } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { robotRunning } from 'stores/states.js';

  // Props
  // svelte-ignore unused-export-let
  export let parent;

  const modalStore = getModalStore();

  async function choose(val) {
    if ($modalStore[0].response) {
      $modalStore[0].response(val);
    }
    modalStore.close();
  }

  function onRobotRunning(mode) {
    if (mode !== 0 && mode !== -1) {
      modalStore.close();
    }
  }

  onMount(() => {
    const unsubscribe = robotRunning.subscribe(onRobotRunning);
    return () => {
      unsubscribe();
    };
  });
</script>

{#if $modalStore[0]}
  <div class="card self-center">
    <header class="card-header text-center font-bold">Start Menu</header>
    <section class="p-4">
      <div class="flex justify-center gap-5">
        <div class="flex flex-col items-center gap-2">
          <button
            type="button"
            class="btn rounded-none focus:!outline-none"
            on:click={() => choose(2)}>
            <div
              class="variant-gradient-primary-secondary w-28 rounded-xl bg-gradient-to-br p-5 text-white">
              <i class="fa-solid fa-image fa-2x text-7xl"></i>
            </div>
          </button>
          <span> Map Creator </span>
        </div>
        <div class="flex flex-col items-center gap-2">
          <button
            type="button"
            class="btn rounded-none focus:!outline-none"
            on:click={() => choose(3)}>
            <div
              class="variant-gradient-error-warning w-28 rounded-xl bg-gradient-to-br p-5 text-white">
              <i class="fa-solid fa-train fa-2x text-7xl"></i>
            </div>
          </button>
          <span> Live App </span>
        </div>
      </div>
    </section>
  </div>
{/if}
