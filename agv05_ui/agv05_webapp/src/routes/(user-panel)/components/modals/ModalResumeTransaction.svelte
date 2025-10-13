<script>
  import { ListBox, ListBoxItem, getModalStore } from '@skeletonlabs/skeleton';

  // Props
  // svelte-ignore unused-export-let
  export let parent;

  let value;

  const modalStore = getModalStore();

  function response(v) {
    if ($modalStore[0].response) {
      $modalStore[0].response(v);
    }
    modalStore.close();
  }

  function confirm() {
    response(value);
  }

  function close() {
    response();
  }
</script>

{#if $modalStore[0] && $modalStore[0].options}
  <div class="fixed inset-0 flex justify-center">
    <div class="card min-w-[400px] self-center p-3">
      <header class="card-header relative h-12 p-3 text-center text-xl font-bold">
        {$modalStore[0].title}
        <button type="button" class="btn-icon absolute right-0 top-0" on:click={close}>
          <i class="fa fa-chevron-up"></i>
        </button>
      </header>
      <hr />
      <section class="grid grid-cols-2 gap-3 p-3">
        <div class="col-span-2">{$modalStore[0].subTitle}</div>
        <ListBox class="col-span-2 space-y-2">
          {#each $modalStore[0].options as o}
            <ListBoxItem bind:group={value} value={o[0]}>{o[1]}</ListBoxItem>
          {/each}
        </ListBox>
        <button
          type="button"
          class="variant-filled btn btn-xl text-center"
          disabled={!value}
          on:click={confirm}>Confirm</button>
        <button
          type="button"
          class="variant-filled-surface btn btn-xl text-center"
          on:click={close}>Cancel</button>
      </section>
    </div>
  </div>
{/if}
