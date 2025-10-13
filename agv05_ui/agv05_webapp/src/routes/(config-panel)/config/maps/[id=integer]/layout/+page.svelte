<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import { getEnv } from 'stores/auth';
  import maps from '$lib/shared/services/config/maps';
  import MapEditorTab from '../../components/MapEditorTab.svelte';
  import MapLayoutEditor from './map/MapLayoutEditor.svelte';
  import MapXLayoutEditor from './mapx/MapXLayoutEditor.svelte';

  export let data;
  let viz;
  let overwritePanel;
  let dirty = false;
  let isOverwrite = false;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  function editorReady(e) {
    viz = e.target.viz;
  }

  function editorDirty() {
    dirty = true;
  }

  function editorSave() {
    save();
  }

  function onForceSubmit() {
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
    if (!viz) {
      return;
    }
    var payload = Object.assign(viz.editor.stage(), {
      modified: data.layout.modified,
      overwrite: overwrite
    });

    maps
      .updateLayout(data.layout.id, payload)
      .then((d) => {
        toastStore.trigger({
          message: maps.successUpdateMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        muteUnload();
        // Prevent using preloaded data
        goto(maps.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        if (e.errorCode == 409) {
          isOverwrite = true;
          setTimeout(() => {
            overwritePanel.scrollIntoView();
          }, 100);
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function muteUnload() {
    dirty = false;
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title={`Map "${data.layout.name}"`} back={['Maps', maps.listUrl()]}>
  <MapEditorTab mapID={data.layout.id} curMode="layout" />
  {#if isOverwrite}
    <div class="grid py-2 lg:grid-cols-2" bind:this={overwritePanel}>
      <aside class="alert variant-filled-error">
        <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
        <div class="alert-message" data-toc-ignore="">
          <h3 class="h3" data-toc-ignore="">Warning</h3>
          <p>
            The content has been
            <a href="${maps.layoutUrl(data.layout.id)}" target="_blank" class="font-bold">
              changed</a>
            while you are editing it.
          </p>
        </div>
        <div class="alert-actions">
          <button type="button" class="force-save variant-filled btn" on:click={onForceSubmit}
            >Force overwrite</button>
        </div>
      </aside>
    </div>
  {/if}
  {#if getEnv('TRACKLESS')}
    <MapXLayoutEditor
      {data}
      on:ready={editorReady}
      on:dirty={editorDirty}
      on:save={editorSave} />
  {:else}
    <MapLayoutEditor
      {data}
      on:ready={editorReady}
      on:dirty={editorDirty}
      on:save={editorSave} />
  {/if}
</ConfigLayout>
