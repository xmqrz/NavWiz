<script>
  import { onMount, onDestroy } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { search } from 'stores/search';
  import SearchModal from './SearchModal.svelte';

  const modalStore = getModalStore();

  search.set(handleSearch);

  function handleSearch(data) {
    const modal = {
      type: 'component',
      component: {
        ref: SearchModal
      },
      ...(data && { meta: { search: data } })
      // response: (r) => {
      //   if (r) {
      //   }
      // }
    };
    modalStore.trigger(modal);
  }

  function handleKeydown(e) {
    if (e.ctrlKey && e.key === 'k') {
      e.preventDefault();
      handleSearch();
    }
  }

  onMount(() => {
    addEventListener('keydown', handleKeydown);
  });

  onDestroy(() => {
    removeEventListener('keydown', handleKeydown);
  });
</script>

<button
  type="button"
  class="variant-filled-secondary bg-secondary-400-500-token btn h-[42px] space-x-4 text-black"
  on:click={() => handleSearch()}>
  <i class="fa-solid fa-magnifying-glass mr-2"></i>
  <small class="hidden lg:inline-block">Ctrl+K</small>
</button>
