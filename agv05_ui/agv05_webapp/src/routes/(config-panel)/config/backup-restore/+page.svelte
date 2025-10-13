<script>
  import * as _ from 'lodash-es';
  import { goto, invalidateAll, afterNavigate } from '$app/navigation';
  import { getToastStore, getModalStore } from '@skeletonlabs/skeleton';

  import SkillsetRestoreImg from 'components/SkillsetRestoreImg.svelte';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import backupRestore from '$lib/shared/services/config/backup-restore';
  import editorF from './restore-advanced/editor';

  let editor;

  function editorReady(e) {
    editor = e.target.editor;
  }

  const toastStore = getToastStore();
  const modalStore = getModalStore();

  const BackupOptions = [
    ['map', 'Map'],
    ['task_template', 'Task Template'],
    ['audio', 'Audio'],
    ['parameter', 'Parameter'],
    ['variable', 'Variable']
  ];

  let restoreMode = '';
  const restoreModeChoices = {
    '': '',
    restore: 'Restore (clear local !!)',
    overwrite: 'Import (conflict: overwrite local !)',
    preserve: 'Import (conflict: keep local)',
    rename: 'Import (conflict: keep both)'
  };

  let restoreFileInput;
  let advancedRestoreMode = false;
  let fileDirty = false;
  let restoreOptions = null;
  let disabledRestoreOptions = [];
  let restoreChoices = [];
  let restoreTree = null;
  let restoreInfo = {};

  let restoreForm = null;

  let backupChoices = [];
  for (const choice of BackupOptions) {
    backupChoices.push(choice[0]);
  }

  afterNavigate(() => {
    restoreFileInput.value = '';
    restoreMode = '';

    advancedRestoreMode = false;
    fileDirty = false;
    restoreChoices = null;
    restoreOptions = null;
    restoreInfo = null;
    restoreTree = null;
  });

  function backupFormSubmit() {
    backupRestore
      .backup({
        backup_choices: backupChoices
      })
      .catch((e) => {
        console.log(e);
      });
  }

  function updateRestoreTree() {
    editor.models.changeMode(restoreMode);
  }

  function restoreFormSubmit() {
    modalStore.trigger({
      type: 'confirm',
      title: 'Please Confirm',
      body: `Are you sure you wish to <strong>${
        advancedRestoreMode ? restoreModeChoices[restoreMode] : 'restore'
      }</strong>?`,
      response: (r) => {
        fileDirty = false;
        if (r) {
          const spinner = {
            type: 'component',
            component: 'modalLoadingSpinner',
            meta: {
              content: 'Restoring...'
            }
          };
          modalStore.trigger(spinner);
          if (advancedRestoreMode) {
            let payload = editor.stage();
            let formData = new FormData(restoreForm);
            formData.append('selected_tree', payload.value);
            formData.append('mode', restoreMode);
            backupRestore
              .advancedRestore(formData)
              .then((_d) => {
                toastStore.trigger({
                  message: backupRestore.successRestoreMsg(),
                  timeout: 3000,
                  hoverable: true
                });
                modalStore.close(spinner);
                goto(backupRestore.backupRestoreUrl()).then(() => invalidateAll());
              })
              .catch((e) => {
                modalStore.close(spinner);
                console.log(e);
              });
          } else {
            let formData = new FormData(restoreForm);
            formData.append('restore_choices', JSON.stringify(restoreChoices));
            backupRestore
              .restore(formData)
              .then((_d) => {
                toastStore.trigger({
                  message: backupRestore.successRestoreMsg(),
                  timeout: 3000,
                  hoverable: true
                });
                modalStore.close(spinner);
                goto(backupRestore.backupRestoreUrl()).then(() => invalidateAll());
              })
              .catch((e) => {
                modalStore.close(spinner);
                console.log(e);
              });
          }
        }
      }
    });
  }

  function clearOptions() {
    fileDirty = true;
    restoreChoices = null;
    restoreOptions = null;
    restoreInfo = null;
    restoreTree = null;
  }

  function uploadFileSubmit() {
    fileDirty = false;
    const spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: {
        content: 'Uploading file...'
      }
    };
    modalStore.trigger(spinner);
    if (advancedRestoreMode) {
      backupRestore
        .getRestoreOptionsTree(restoreForm)
        .then((d) => {
          if (d.error) {
            toastStore.trigger({
              message: d.error,
              timeout: 3000,
              hoverable: true
            });
          } else {
            restoreTree = d.options_tree;
            restoreInfo = JSON.parse(d.restore_info);
            restoreOptions = null;
            const editorDiv = document.getElementById('advanced-restore-editor');
            if (editorDiv) {
              editorDiv.remove();
            }
          }
        })
        .finally(() => {
          modalStore.close(spinner);
        });
    } else {
      backupRestore
        .getRestoreChoices(restoreForm)
        .then((d) => {
          if (d.error) {
            toastStore.trigger({
              message: d.error,
              timeout: 3000,
              hoverable: true
            });
          } else {
            restoreOptions = d.restore_choices;
            disabledRestoreOptions = JSON.parse(d.disabled_choices);
            restoreInfo = JSON.parse(d.restore_info);
            restoreTree = null;
            const editorDiv = document.getElementById('advanced-restore-editor');
            if (editorDiv) {
              editorDiv.remove();
            }
          }
        })
        .finally(() => {
          modalStore.close(spinner);
        });
    }
  }

  function onCancelRestore() {
    goto(backupRestore.backupRestoreUrl()).then(() => invalidateAll());
  }
</script>

<ConfigLayout title="Backup Restore">
  <div>
    <span class="block text-2xl">Backup</span>
    <form on:submit={backupFormSubmit}>
      <div class="grid w-full grid-cols-7 gap-4">
        <div class="col-span-2 text-right">Check to backup</div>
        <div class="col-span-5 space-y-2">
          <div>
            {#each BackupOptions as choice}
              <label class="space-x-2">
                <input
                  class="checkbox"
                  type="checkbox"
                  value={choice[0]}
                  bind:group={backupChoices} />
                <span>{choice[1]}</span>
              </label>
            {/each}
          </div>
          <div>
            <button type="submit" class="variant-filled-primary btn">
              Backup Settings to File
            </button>
            <a class="variant-filled-primary btn" href={backupRestore.advancedBackupUrl()}>
              Advanced Backup
            </a>
          </div>
        </div>
      </div>
    </form>
  </div>
  <hr class="my-4" />
  <div class="space-y-4">
    <span class="block text-2xl">Restore</span>
    <form
      bind:this={restoreForm}
      on:submit|stopPropagation|preventDefault={restoreFormSubmit}
      class="space-y-2">
      <div class="grid w-full grid-cols-7 gap-4">
        <div class="col-span-2 text-right">Restore File</div>
        <div class="card col-span-4 space-y-2 rounded-2xl p-4">
          <input
            type="file"
            name="restore_file"
            accept=".gz"
            bind:this={restoreFileInput}
            required
            on:change={clearOptions} />
          <label class="space-x-2">
            <input class="checkbox" type="checkbox" bind:checked={advancedRestoreMode} />
            <span>Advanced restore.</span>
          </label>
          <button
            type="button"
            class="variant-filled-primary btn"
            disabled={!fileDirty}
            on:click={uploadFileSubmit}>Upload</button>
        </div>
      </div>
      {#if (restoreOptions && restoreOptions.length > 0) || restoreTree}
        <div class="grid w-full grid-cols-7 gap-4">
          <div class="col-span-2 text-right">Restore Choices</div>
          <div class="col-span-4 space-y-2">
            <div class="card grid grid-cols-4 space-y-2 rounded-2xl p-4">
              {#if restoreOptions && restoreOptions.length > 0}
                <div class="col-span-3">
                  {#each restoreOptions as option}
                    <label class="space-x-2">
                      <input
                        class="checkbox"
                        type="checkbox"
                        value={option}
                        name="restore_choices"
                        disabled={disabledRestoreOptions.includes(option)}
                        bind:group={restoreChoices} />
                      <span>{option}</span>
                    </label>
                  {/each}
                </div>
              {:else if restoreTree}
                <div class="col-span-3 space-y-2 p-4">
                  Restore Mode
                  <select
                    class="select px-7 rounded-token"
                    name="mode"
                    bind:value={restoreMode}
                    on:change={updateRestoreTree}
                    required>
                    {#each Object.entries(restoreModeChoices) as [v, display]}
                      <option value={v}>{display}</option>
                    {/each}
                  </select>
                  <div
                    class="editor"
                    id="advanced-restore-editor"
                    on:ready={editorReady}
                    use:editorF={restoreTree}>
                  </div>
                </div>
              {/if}
              {#if restoreInfo}
                <SkillsetRestoreImg
                  src={restoreInfo.skillset_img}
                  name={restoreInfo.name}
                  version={restoreInfo.version} />
              {/if}
              <button
                type="button"
                class="variant-filled-surface btn mr-1"
                on:click={onCancelRestore}>
                Cancel
              </button>
              <button type="submit" class="variant-filled-primary btn">Restore</button>
            </div>
          </div>
        </div>
      {/if}
    </form>
  </div>
</ConfigLayout>
