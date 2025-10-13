<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import maps from '$lib/shared/services/config/maps';
  import MapEditorTab from '../../components/MapEditorTab.svelte';

  export let data;

  const toastStore = getToastStore();

  let form;
  let name = data.map.name;
  let error = {};

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
      .update(data.map.id, payload)
      .then((d) => {
        toastStore.trigger({
          message: maps.successUpdateMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        // Prevent using preloaded data
        goto(maps.listUrl()).then(() => invalidateAll());
      })
      .catch(async (e) => {
        if (e.errorCode === 400) {
          const d = await e.cause.json();
          if (d.name && Array.isArray(d.name)) {
            d.name = d.name[0];
          }
          error = d;
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title={`Map "${data.map.display_name}"`} back={['Maps', maps.listUrl()]}>
  <MapEditorTab mapID={data.map.id} curMode="edit" />
  <form bind:this={form} class="w-full space-y-4 lg:w-3/4">
    <label class="grid grid-cols-4">
      <span class="pr-4 text-right text-lg font-semibold" class:text-red-500={error.name}
        >Name</span>
      <div class="col-span-3">
        <input
          class="input"
          class:input-error={error.name}
          type="text"
          bind:value={name}
          required />
      </div>
      {#if error.name}
        <span class="col-span-3 col-start-2 text-red-500">{error.name}</span>
      {/if}
    </label>
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Save
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
