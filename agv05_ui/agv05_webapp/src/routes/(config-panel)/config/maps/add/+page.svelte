<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import maps from '$lib/shared/services/config/maps';

  export let data;
  let form;
  let name;
  let error = {};

  const title = `Add new map${data.map ? ' (clone of map "' + data.map.name + '")' : ''}`;

  const toastStore = getToastStore();

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    save();
  }

  function save() {
    let payload = {
      name: name
    };
    maps
      .add(data.map ? data.map.id : undefined, payload)
      .then((d) => {
        toastStore.trigger({
          message: maps.successAddMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        goto(maps.layoutUrl(d.id));
      })
      .catch(async (e) => {
        if (e.errorCode === 400) {
          const c = await e.cause.json();
          error = c;
        }
        if (e.errorCode === 404) {
          toastStore.trigger({
            background: 'variant-filled-error',
            message: 'Clone map not found.',
            timeout: 3000,
            hoverable: true
          });
          // Prevent using preloaded data
          goto(maps.listUrl()).then(() => invalidateAll());
        }
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout {title} back={['Maps', maps.listUrl()]}>
  <form bind:this={form}>
    <label class="label grid grid-cols-4 items-center p-3">
      <span
        class:text-error-600={'name' in error}
        class="px-4 text-right text-lg font-semibold">
        Name
      </span>
      <input
        class:input-error={'name' in error}
        class="input col-span-3"
        type="text"
        bind:value={name}
        required />
      {#if 'name' in error}
        <span class="col-span-3 col-start-2 font-semibold text-red-600">
          {error.name}
        </span>
      {/if}
    </label>
    <div class="grid grid-cols-4 p-3 pt-9">
      <button
        type="submit"
        class="variant-filled-primary btn col-span-3 col-start-2 w-max"
        on:click|preventDefault={onSubmit}>
        Save and continue editing layout
      </button>
    </div>
  </form>
</ConfigLayout>
