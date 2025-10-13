<script>
  import * as _ from 'lodash-es';

  import backupRestore from '$lib/shared/services/config/backup-restore';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import editorF from './editor';

  export let data;

  let form;
  let editor;

  function editorReady(e) {
    editor = e.target.editor;
  }

  const BackupOptions = [
    ['map', 'Map'],
    ['task_template', 'Task Template'],
    ['audio', 'Audio'],
    ['parameter', 'Parameter'],
    ['variable', 'Variable']
  ];

  let backup_choices = [];
  for (const choice of BackupOptions) {
    backup_choices.push(choice[0]);
  }

  function onSubmit() {
    save();
  }

  function save() {
    let payload = editor.stage();
    backupRestore
      .advancedBackup({
        selected_tree: payload.value
      })
      .catch((e) => {
        console.log(e);
      });
  }
</script>

<ConfigLayout title="Advanced Backup" back={['Back', backupRestore.backupRestoreUrl()]}>
  <form bind:this={form} on:submit={onSubmit} class="space-y-2">
    <div class="editor" on:ready={editorReady} use:editorF={data}></div>
    <div class="grid grid-cols-12">
      <span class="col-span-2"></span>
      <div class="col-span-10">
        <button type="submit" class="variant-filled-primary btn">
          Backup Settings to File
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
