<script>
  import { readable } from 'svelte/store';
  import { getModalStore } from '@skeletonlabs/skeleton';

  // Props
  // svelte-ignore unused-export-let
  export let parent;
  let content = readable();

  const modalStore = getModalStore();

  function updateContent(store) {
    content = readable();
    if (!store || !store.meta) {
      return;
    }
    if (store.meta.content) {
      content = readable(store.meta.content);
    }
    if (store.meta.contentStore) {
      content = store.meta.contentStore;
    }
  }

  $: updateContent($modalStore[0]);
</script>

<svelte:window on:keydown|capture|stopPropagation />

{#if $modalStore[0] && $modalStore[0].meta}
  <div class="fixed inset-0 flex justify-center">
    <div
      class="flex flex-col justify-center self-center rounded-3xl bg-black px-14 py-10 opacity-70">
      <svg width="24" height="24" viewBox="0 0 24 24" class="h-5 w-5 animate-spin self-center">
        <path
          d="M12,1A11,11,0,1,0,23,12,11,11,0,0,0,12,1Zm0,19a8,8,0,1,1,8-8A8,8,0,0,1,12,20Z"
          opacity=".25"
          stroke="white"
          fill="white" /><path
          stroke="white"
          fill="white"
          d="M10.14,1.16a11,11,0,0,0-9,8.92A1.59,1.59,0,0,0,2.46,12,1.52,1.52,0,0,0,4.11,10.7a8,8,0,0,1,6.66-6.61A1.42,1.42,0,0,0,12,2.69h0A1.57,1.57,0,0,0,10.14,1.16Z" />
      </svg>
      <!-- eslint-disable-next-line svelte/no-at-html-tags -->
      <span class="pt-3 text-white">{@html $content || 'Loading...'}</span>
    </div>
  </div>
{/if}
