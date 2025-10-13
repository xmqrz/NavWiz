<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import MapParamEditorTab from '../components/MapParamEditorTab.svelte';
  import maps from '$lib/shared/services/config/maps';
  import editorF from './editor.js';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let form;
  let overwritePanel;
  let isOverwrite = false;
  let dirty = false;
  let editor;

  function editorDirty() {
    dirty = true;
  }

  function editorReady(e) {
    editor = e.target.editor;
  }

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
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
    let payload = Object.assign(editor.stage(), {
      modified: data.teleport.modified,
      overwrite: overwrite
    });
    maps
      .updateTeleport(payload)
      .then(() => {
        toastStore.trigger({
          message: maps.successUpdateTeleportMsg(),
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

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title="Edit map teleportation" back={['Maps', maps.listUrl()]}>
  <MapParamEditorTab curMode="teleport" />
  {#if isOverwrite}
    <div class="grid py-2 lg:grid-cols-2" bind:this={overwritePanel}>
      <aside class="alert variant-filled-error">
        <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
        <div class="alert-message" data-toc-ignore="">
          <h3 class="h3" data-toc-ignore="">Warning</h3>
          <p>
            The content has been
            <a href="${maps.teleportUrl()}" target="_blank" class="font-bold">changed</a>
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
  <form bind:this={form} on:submit|preventDefault novalidate>
    <div class="editor" on:dirty={editorDirty} on:ready={editorReady} use:editorF={data}></div>
    <div class="p-3 pt-9">
      <button
        type="submit"
        class="variant-filled-primary btn"
        disabled={!dirty}
        on:click|preventDefault={onSubmit}>
        Submit
      </button>
    </div>
  </form>
</ConfigLayout>
