<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { beforeNavigate, invalidateAll } from '$app/navigation';
  import { page } from '$app/stores';

  import ac from '$lib/shared/services/config/agv-activities';
  import Editor from './Editor.svelte';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let form;
  let overwritePanel;
  let isOverwrite = false;

  let activityLabels = data.configuration.labels;
  let activities = data.activities;
  let dirty = false;

  function getConfiguration() {
    return {
      labels: activityLabels
    };
  }

  function updateActivityLabel(code, label) {
    if (!label) {
      delete activityLabels[code];
    } else {
      activityLabels[code] = label;
    }
    triggerDirty();
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
      value: JSON.stringify(getConfiguration()),
      modified: data.modified,
      overwrite: overwrite
    };
    ac.updateConfig(payload)
      .then(() => {
        toastStore.trigger({
          message: ac.successUpdateConfigMsg(),
          timeout: 3000,
          hoverable: true
        });
        isOverwrite = false;
        dirty = false;
        invalidateAll();
      })
      .catch((e) => {
        if (e.errorCode === 409) {
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

<div class="space-y-4 py-6">
  {#if isOverwrite}
    <div class="grid py-2 lg:grid-cols-2" bind:this={overwritePanel}>
      <aside class="alert variant-filled-error">
        <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
        <div class="alert-message" data-toc-ignore="">
          <h3 class="h3" data-toc-ignore="">Warning</h3>
          <p>
            The content has been
            <a href={$page.url.pathname} target="_blank" class="font-bold">changed</a>
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
  <form bind:this={form} on:submit|preventDefault>
    <Editor {activities} {activityLabels} {updateActivityLabel} />
    <div class="px-3 pt-7">
      <button
        type="submit"
        class="variant-filled-primary btn"
        disabled={!dirty}
        on:click|preventDefault={onSubmit}>
        Submit
      </button>
    </div>
  </form>
</div>
