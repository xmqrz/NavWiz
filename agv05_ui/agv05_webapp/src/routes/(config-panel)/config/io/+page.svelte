<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { beforeNavigate, invalidateAll } from '$app/navigation';
  import { page } from '$app/stores';

  import ioService from '$lib/shared/services/config/io';
  import Editor from './Editor.svelte';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let form;
  let overwritePanel;
  let isOverwrite = false;

  const PORT = 8;
  const PIN = 16;

  let outputName = [];
  let inputName = [];
  let dirty = false;

  for (let i = 0; i < PORT; i++) {
    let outputN = [];
    let inputN = [];
    for (let j = 0; j < PIN; j++) {
      outputN[j] = '';
      inputN[j] = '';
    }
    outputName[i] = outputN;
    inputName[i] = inputN;
  }

  let ioName;
  try {
    ioName = JSON.parse(data.io.value);
  } catch (e) {
    console.log('IO name parse error.');
  }

  if (ioName) {
    if (ioName.output_name && Array.isArray(ioName.output_name)) {
      for (let i = 0; i < ioName.output_name.length && i < PORT; i++) {
        let names = ioName.output_name[i];
        if (!names || !Array.isArray(names)) {
          continue;
        }
        for (let j = 0; j < PIN; j++) {
          outputName[i][j] = names[j] || '';
        }
      }
    }

    if (ioName.input_name && Array.isArray(ioName.input_name)) {
      for (let i = 0; i < ioName.input_name.length && i < PORT; i++) {
        let names = ioName.input_name[i];
        if (!names || !Array.isArray(names)) {
          continue;
        }
        for (let j = 0; j < PIN; j++) {
          inputName[i][j] = names[j] || '';
        }
      }
    }
  }

  function update(port, pin, isOutput, name) {
    if (isOutput) {
      outputName[port][pin] = name;
    } else {
      inputName[port][pin] = name;
    }
    triggerDirty();
  }

  function triggerDirty() {
    dirty = true;
  }

  function stageData() {
    return {
      input_name: inputName,
      output_name: outputName
    };
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
      value: JSON.stringify(stageData()),
      modified: data.io.modified,
      overwrite: overwrite
    };
    ioService
      .update(payload)
      .then(() => {
        toastStore.trigger({
          message: ioService.successUpdateMsg(),
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

<ConfigLayout title="I/O">
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
    <Editor {PORT} {PIN} {outputName} {inputName} {update} />
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
</ConfigLayout>
