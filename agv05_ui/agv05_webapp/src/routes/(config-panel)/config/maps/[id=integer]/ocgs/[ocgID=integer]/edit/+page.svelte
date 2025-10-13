<script>
  import { getToastStore, getModalStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import MapEditorTab from '../../../../components/MapEditorTab.svelte';
  import maps from '$lib/shared/services/config/maps';
  import editorF from './editor.js';

  export let data;

  const toastStore = getToastStore();
  const modalStore = getModalStore();

  let form;
  let viz;
  let dirty = false;
  let replace = false;
  let newName;
  let clearName = false;

  $: if (!replace) {
    clearName = false;
  }

  $: if (clearName) {
    newName = undefined;
  }

  function editorReady(e) {
    viz = e.target.viz;
  }

  function editorDirty() {
    dirty = true;
  }

  function triggerDirty() {
    if (!viz) {
      return;
    }
    viz.models.externalTriggerDirty();
  }

  function muteUnload() {
    dirty = false;
  }

  function editorSave() {
    if (!viz) {
      return;
    }
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    let payload = createPayload();
    let fetch;
    if (replace) {
      fetch = maps.updateOcg(data.mapID, data.ocg.id, payload);
    } else {
      fetch = maps.addOcg(data.mapID, payload);
    }

    fetch
      .then((d) => {
        toastStore.trigger({
          message: maps.successUpdateOcgMsg(d.display_name),
          timeout: 3000,
          hoverable: true
        });
        muteUnload();
        // Prevent using preloaded data
        goto(maps.ocgsUrl(data.mapID)).then(() => invalidateAll());
      })
      .catch(async (e) => {
        if (e.errorCode == 409) {
          // TODO: add overwirte
          // isOverwrite = true;
          // setTimeout(() => {
          //   overwritePanel.scrollIntoView();
          // }, 100);
          // return;
        }
        if (e.errorCode == 400) {
          const d = await e.cause.json();
          console.log(d);
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function createPayload() {
    let stage = viz.editor.stage();

    let payload = Object.assign(
      {
        clear_name: clearName
      },
      stage
    );

    if (newName) {
      payload['name'] = newName;
    }

    return payload;
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title={`Map "${data.ocg.map_display_name}"`} back={['Maps', maps.listUrl()]}>
  <MapEditorTab mapID={data.mapID} curMode="ocg" />
  <div class="flex space-x-5 py-5">
    <a href={maps.ocgsUrl(data.mapID)} class="variant-ghost btn btn-sm">
      <i class="fa-solid fa-chevron-left mr-2"></i>
      Raw Maps
    </a>
    <h1 class="text-2xl tracking-widest">{`Edit raw map "${data.ocg.display_name}"`}</h1>
  </div>
  <hr />
  <form bind:this={form} class="agv05x-ocg relative pt-3" on:submit|preventDefault novalidate>
    <div class="container space-y-5 py-10">
      <label class="label grid grid-cols-4 items-center">
        <span class="pr-4 text-right text-lg font-semibold">New name</span>
        <input
          placeholder={clearName ? '(cleared)' : replace ? '(no change)' : '(unnamed)'}
          class="input col-span-3 px-7"
          type="text"
          name="name"
          on:change={triggerDirty}
          bind:value={newName}
          disabled={clearName} />
      </label>
      <div class="label grid grid-cols-4">
        <label class="col-span-3 col-start-2 flex items-center space-x-2">
          <input
            class="checkbox mr-1"
            type="checkbox"
            name="clear_name"
            on:change={triggerDirty}
            bind:checked={clearName}
            disabled={!replace} />
          <span class="pr-4 text-lg font-semibold">Clear name</span>
        </label>
      </div>
      <div class="label grid grid-cols-4">
        <label class="col-span-3 col-start-2 flex items-center space-x-2">
          <input class="checkbox mr-1" type="checkbox" bind:checked={replace} />
          <span class="pr-4 text-lg font-semibold">
            Replace the original raw map instead of saving a new copy.
          </span>
        </label>
      </div>
    </div>
    <div
      class="editor"
      editable="true"
      dynamic={data.ocg.dynamic || 'false'}
      resizable="true"
      grid="true"
      on:ready={editorReady}
      on:dirty={editorDirty}
      on:save={editorSave}
      use:editorF={[data, modalStore]}>
    </div>
    <div class="title-info">
      <strong>Raw Map:</strong>
      {data.ocg.display_name}<br />
      <strong>Map:</strong>
      {data.ocg.map_display_name}
    </div>
  </form>
</ConfigLayout>

<style global>
  @import '$lib/styles/map-ocg.css';
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
  @import '$lib/styles/agv05-toolbar.css';
</style>
