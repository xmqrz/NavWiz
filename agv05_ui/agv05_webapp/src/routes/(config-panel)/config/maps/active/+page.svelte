<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';
  import { onMount, onDestroy } from 'svelte';

  import MapParamEditorTab from '../components/MapParamEditorTab.svelte';
  import maps from '$lib/shared/services/config/maps';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let form;
  let overwritePanel;
  let isOverwrite = false;
  let dirty = false;
  let ready = false;
  let mapList = [];
  let mounted;

  let activeMaps;
  if (Array.isArray(data.active.results)) {
    activeMaps = data.active.results.map((d) => d.id);
  }

  function triggerDirty() {
    dirty = true;
  }

  function onSubmit() {
    save();
  }

  function onForceSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    modalStore.trigger({
      type: 'confirm',
      title: 'Overwrite changes',
      body: 'Overwrite the other changes?',
      response: (r) => {
        if (!r) {
          return;
        }
        save(true);
      }
    });
  }

  function save(overwrite = false) {
    let payload = {
      value: activeMaps.join(','),
      modified: data.active.modified,
      overwrite: overwrite
    };
    maps
      .updateActive(payload)
      .then(() => {
        toastStore.trigger({
          message: maps.successUpdateActiveMsg(),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        goto(maps.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        if (e.errorCode == 409) {
          isOverwrite = true;
          // let it draw first and then scrollIntoView
          setTimeout(() => {
            overwritePanel.scrollIntoView();
          }, 100);
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  onMount(() => {
    mounted = true;
    maps.getAllList(function (response, results, last) {
      mapList = results;
      ready = last;
      return !mounted;
    });
  });

  onDestroy(() => {
    mounted = false;
  });

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title="Edit active map" back={['Maps', maps.listUrl()]}>
  <MapParamEditorTab curMode="active" />
  {#if isOverwrite}
    <div class="grid py-2 lg:grid-cols-2" bind:this={overwritePanel}>
      <aside class="alert variant-filled-error">
        <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
        <div class="alert-message" data-toc-ignore="">
          <h3 class="h3" data-toc-ignore="">Warning</h3>
          <p>
            The content has been
            <a href="${maps.activeUrl()}" target="_blank" class="font-bold">changed</a>
            while you are editing it.
          </p>
        </div>
        <div class="alert-actions">
          <button type="button" class="force-save variant-filled btn" on:click={onForceSubmit}>
            Force overwrite
          </button>
        </div>
      </aside>
    </div>
  {/if}
  <form bind:this={form} class="w-full lg:w-3/4">
    <div class="grid grid-cols-4 space-y-2">
      <span class="pr-4 text-right text-lg font-semibold">Active Map</span>
      <div class="col-span-3">
        {#each mapList as map}
          <label class="flex items-center space-x-2">
            <input
              class="checkbox"
              type="checkbox"
              value={map.id}
              bind:group={activeMaps}
              on:change={triggerDirty} />
            <p>{map.name}</p>
          </label>
        {/each}
      </div>
    </div>
    <div class="mt-7 grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          disabled={!dirty || !ready}
          on:click|preventDefault={onSubmit}>
          Submit
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
