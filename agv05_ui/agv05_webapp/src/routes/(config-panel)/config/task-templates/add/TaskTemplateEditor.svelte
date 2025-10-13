<script>
  import { createEventDispatcher } from 'svelte';
  import { beforeNavigate } from '$app/navigation';

  import taskTemplates from '$lib/shared/services/config/task-templates';
  import editor from './editor.js';

  export let data;
  export let isOverwrite = false;
  export let editable = true;
  export let preserve = false;
  export let taskTemplateUrl = taskTemplates.editUrl('999999999');

  const dispatch = createEventDispatcher();

  let form;
  let taskTemplate = data.taskTemplate || {};
  let name = taskTemplate.name;
  let nameInput;
  let category = taskTemplate.category;
  let active = taskTemplate.is_active;
  let topLvl = taskTemplate.is_top_level;
  let createSuspended = taskTemplate.create_suspended;
  let dirty = false;
  let viz;
  let error = {};

  export function muteUnload() {
    dirty = false;
  }

  export async function saveErrorCB(e) {
    if (e.errorCode === 400) {
      const d = await e.cause.json();
      if (d.name && Array.isArray(d.name)) {
        d.name = d.name[0];
        // let it draw first and then scrollIntoView
        setTimeout(() => {
          nameInput.scrollIntoView();
        }, 100);
      }
      error = d;
      return true;
    }
  }

  function resetOutcomes(isTopLvl) {
    if (!viz) {
      return;
    }
    viz.models.setTopLvl(isTopLvl);
  }
  $: resetOutcomes(topLvl);
  function updateCreateSuspended(isCreateSuspended) {
    if (!viz) {
      return;
    }
    viz.models.setCreateSuspended(isCreateSuspended);
  }
  $: updateCreateSuspended(createSuspended);

  function editorReady(e) {
    viz = e.target.viz;
  }

  function editorDirty() {
    dirty = true;
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
    dispatch('save', payload);
  }

  function editorForceSave() {
    if (!viz) {
      return;
    }
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    let payload = createPayload();
    dispatch('force-save', payload);
  }

  function createPayload() {
    let stage = viz.editor.stage();
    return Object.assign({}, stage, {
      name: name,
      category: category,
      is_active: active,
      is_top_level: topLvl,
      create_suspended: createSuspended
    });
  }

  function triggerDirty() {
    if (!viz) {
      return;
    }
    viz.models.externalTriggerDirty();
  }

  function overwriteUpdate(state) {
    if (!viz || !state) {
      return;
    }
    viz.models.enableOverwrite();
  }
  $: overwriteUpdate(isOverwrite);

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<form action="" bind:this={form} on:submit|preventDefault novalidate>
  <div class="grid lg:grid-cols-2">
    <div>
      <label class="label p-3">
        <span class="font-medium" class:text-red-500={error.name}>Name</span>
        <input
          class="input"
          class:input-error={error.name}
          type="text"
          bind:value={name}
          disabled={!editable || !viz}
          on:change={triggerDirty}
          bind:this={nameInput}
          required />
        {#if error.name}
          <span class="text-red-500">{error.name}</span>
        {/if}
      </label>
      <label class="label p-3">
        <span class="font-medium">Category</span>
        <input
          class="input"
          type="text"
          bind:value={category}
          on:change={triggerDirty}
          disabled={!editable || !viz} />
      </label>
    </div>
    <div class="pt-3 lg:pl-20">
      <span class="p-3 font-medium">Options</span>
      <div class="space-y-5 p-3">
        <label class="flex items-center space-x-2">
          <input
            class="checkbox"
            type="checkbox"
            bind:checked={active}
            on:change={triggerDirty}
            disabled={!editable} />
          <p>Active</p>
        </label>
        <label class="flex items-center space-x-2">
          <input
            class="checkbox"
            type="checkbox"
            bind:checked={topLvl}
            disabled={!editable || !viz} />
          <p>Top-level template</p>
        </label>
        <label class="flex items-center space-x-2">
          <input
            class="checkbox"
            type="checkbox"
            disabled={!editable || !topLvl || !viz}
            bind:checked={createSuspended} />
          <p>Suspend task initially</p>
        </label>
      </div>
    </div>
  </div>
  <div
    class="editor"
    editable={editable ? 'true' : 'false'}
    preserve={preserve ? 'true' : 'false'}
    {taskTemplateUrl}
    resizable="true"
    on:ready={editorReady}
    on:dirty={editorDirty}
    on:save={editorSave}
    on:force-save={editorForceSave}
    use:editor={data}>
    <div class="title-info">
      <strong>Task Template:</strong>
      {data.taskTemplate?.name || '(new)'}
    </div>
  </div>
</form>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/agv05-toolbar.css';
</style>
