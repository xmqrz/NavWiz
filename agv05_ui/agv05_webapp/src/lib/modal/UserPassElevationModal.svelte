<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';

  // svelte-ignore unused-export-let
  export let parent;

  const modalStore = getModalStore();

  let username = '';
  let password = '';

  function onKeyboard(event) {
    if (event.keyCode === 13) {
      // enter key
      onOk();
      event.preventDefault();
    }
  }

  onMount(() => {
    document.addEventListener('keydown', onKeyboard);

    return () => {
      document.removeEventListener('keydown', onKeyboard);
    };
  });

  function onOk() {
    if (!username || !password) {
      return;
    }
    if ($modalStore[0].response) {
      const b = window.btoa(`${username}:${password}`);
      $modalStore[0].response(`Basic ${b}`);
    }
    modalStore.close();
  }

  function onCancel() {
    if ($modalStore[0].response) {
      $modalStore[0].response();
    }
    modalStore.close();
  }
</script>

{#if $modalStore[0]}
  <div class="card p-3">
    <section class="grid gap-3 p-3">
      <div class="text-center font-bold">This action require elevation</div>
      <div class="text-center text-xl">Username</div>
      <div class="text-center">
        <input type="text" class="input rounded text-xl" bind:value={username} />
      </div>
      <div class="text-center text-xl">Password</div>
      <div class="text-center">
        <input type="password" class="input rounded text-xl" bind:value={password} />
      </div>
    </section>
    <footer class="card-footer grid grid-cols-2 gap-3 p-3">
      <button
        type="button"
        disabled={!username || !password}
        class="variant-filled btn btn-xl text-center"
        on:click={onOk}>
        Ok
      </button>
      <button
        type="button"
        class="variant-filled-surface btn btn-xl text-center"
        on:click={onCancel}>
        Cancel
      </button>
    </footer>
  </div>
{/if}
