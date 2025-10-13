<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import groups from '$lib/shared/services/config/groups';

  export let data;

  let dirty = false;
  const toastStore = getToastStore();

  let name = data.name;
  let form;
  let error = {};

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    groups
      .add({
        name: name
      })
      .then((d) => {
        toastStore.trigger({
          message: groups.successAddMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(groups.listUrl()).then(() => invalidateAll());
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

  function triggerDirty() {
    if (!dirty) {
      dirty = true;
    }
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title={`Add new group`} back={['Groups', groups.listUrl()]} validation={false}>
  <form bind:this={form} action="" class="w-full lg:w-3/4">
    <label class="grid grid-cols-4 p-3 px-10">
      <span class="pr-4 text-right text-lg font-semibold" class:text-red-500={error.name}>
        Name
      </span>
      <input
        class="input col-span-3"
        class:input-error={error.name}
        type="text"
        required
        bind:value={name}
        on:change={triggerDirty} />
      {#if error.name}
        <span class="col-span-3 col-start-2 text-red-500">{error.name}</span>
      {/if}
    </label>
    <div class="mt-7 grid grid-cols-4 px-10">
      <div class="col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Submit
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
