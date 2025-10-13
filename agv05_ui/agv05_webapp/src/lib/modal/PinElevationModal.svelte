<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';

  // svelte-ignore unused-export-let
  export let parent;

  const PIN_LENGTH = 6;
  const modalStore = getModalStore();

  let value = '';

  function onNum(n) {
    if (value.length >= PIN_LENGTH) {
      return;
    }
    value = value + n;
  }

  function onBackspace() {
    if (value.length <= 0) {
      return;
    }
    value = value.slice(0, -1);
  }

  function onClear() {
    value = '';
  }

  function onClick() {
    onNum(this.innerText);
  }

  function onKeyboard(event) {
    if (event.keyCode >= 48 && event.keyCode <= 57) {
      // number key
      onNum(event.key);
      event.preventDefault();
    }

    if (event.keyCode === 8) {
      // backspace key
      onBackspace();
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
    if (value.length !== PIN_LENGTH) {
      return;
    }
    if ($modalStore[0].response) {
      $modalStore[0].response(`Pin ${value}`);
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
    <section class="grid grid-cols-3 gap-3 p-3">
      <div class="col-span-3 text-center font-bold">This action require elevation</div>
      <div class="col-span-3 text-center text-xl">Enter Pin Number</div>
      <div class="col-span-3 text-center">
        <input type="password" class="input rounded text-xl" disabled bind:value />
      </div>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>1</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>2</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>3</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>4</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>5</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>6</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>7</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>8</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>9</button>
      <button type="button" class="btn btn-xl text-center" on:click={onClick}>0</button>
      <button type="button" class="btn btn-xl col-span-2 text-center" on:click={onClear}>
        Clear
      </button>
    </section>
    <footer class="card-footer grid grid-cols-2 gap-3 p-3">
      <button
        type="button"
        disabled={value.length !== PIN_LENGTH}
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
