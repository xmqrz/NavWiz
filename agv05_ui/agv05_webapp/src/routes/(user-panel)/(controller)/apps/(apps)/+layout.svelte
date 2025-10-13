<script>
  import { page } from '$app/stores';
  import { fly, fade } from 'svelte/transition';
</script>

{#key $page.route.id}
  <div class="fixed inset-0 z-10 bg-black opacity-50" transition:fade={{ duration: 150 }}>
  </div>
  <div class="fixed inset-0 z-10 p-4 shadow-2xl">
    <div
      class="bg-surface-50-900-token h-full w-full overflow-hidden rounded rounded-t-2xl"
      transition:fly={{ duration: 150, opacity: 0, x: 0, y: 100 }}>
      <slot />
    </div>
  </div>
{/key}
