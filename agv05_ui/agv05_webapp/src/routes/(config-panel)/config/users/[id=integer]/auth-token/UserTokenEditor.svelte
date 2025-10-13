<script>
  import { createEventDispatcher } from 'svelte';

  export let data;
  const dispatch = createEventDispatcher();

  function onRegen() {
    dispatch('regen');
  }
</script>

<form action="">
  <div class="container grid">
    <div class="grid grid-cols-4 items-center p-3">
      <span class="pr-4 text-right text-lg font-semibold">Auth Token</span>
      <div class="input col-span-3 px-4 py-2 pl-5 opacity-70">{data.authToken}</div>
    </div>
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onRegen}>
          Regenerate Auth Token
        </button>
      </div>
    </div>
  </div>
</form>
