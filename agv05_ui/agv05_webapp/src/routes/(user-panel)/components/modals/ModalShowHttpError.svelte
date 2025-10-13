<script>
  import { getModalStore } from '@skeletonlabs/skeleton';

  // Props
  export let parent;

  const modalStore = getModalStore();
</script>

{#if $modalStore[0]}
  <div class="card self-center p-4">
    <header class="card-header text-center font-bold">
      {$modalStore[0].title ?? 'HTTP Error'}
    </header>
    <section class="p-4">
      {#if $modalStore[0].meta}
        {#if $modalStore[0].meta.http}
          {#if $modalStore[0].meta.http.status === 403}
            'The current user does not have the permission to perform this operation.'
          {:else if $modalStore[0].meta.http.status === 401}
            {#if window.agvPanelToken}
              'Invalid pin number.'
            {:else}
              'Invalid username or password.'
            {/if}
          {:else if $modalStore[0].meta.http.data && $modalStore[0].meta.http.data.detail}
            {$modalStore[0].meta.http.data.detail}
          {/if}
        {/if}
      {/if}
    </section>
    <footer class="modal-footer {parent.regionFooter}">
      <button type="button" class="btn {parent.buttonNeutral}" on:click={parent.onClose}>
        {parent.buttonTextCancel}
      </button>
    </footer>
  </div>
{/if}
