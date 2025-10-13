<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import TaskTemplateEditor from '../../add/TaskTemplateEditor.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let editor;
  let isOverwrite = false;
  let id = data.taskTemplate.id;

  function editorSave(e) {
    // TODO: how to make this similar to successmixin.
    var payload = Object.assign({}, e.detail, {
      modified: data.taskTemplate.modified
    });
    taskTemplates
      .update(id, payload)
      .then((d) => {
        toastStore.trigger({
          message: taskTemplates.successUpdateMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        editor.muteUnload();
        // Prevent using preloaded data
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        if (e.errorCode == 409) {
          isOverwrite = true;
          return;
        }
        if (editor.saveErrorCB(e)) {
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function editorForceSave(e) {
    modalStore.trigger({
      type: 'confirm',
      title: 'Overwrite changes',
      body: 'Overwrite the other changes?',
      response: (r) => {
        if (!r) {
          return;
        }
        _editorForceSave(e);
      }
    });
  }

  function _editorForceSave(e) {
    var payload = Object.assign({}, e.detail, {
      overwrite: true
    });
    taskTemplates
      .update(id, payload)
      .then((d) => {
        toastStore.trigger({
          message: taskTemplates.successUpdateMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        editor.muteUnload();
        // Prevent using preloaded data
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        if (e.errorCode == 409) {
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title={`Edit task template "${data.taskTemplate.name}"`}
  back={['Task Templates', taskTemplates.listUrl()]}>
  <TaskTemplateEditor
    {data}
    on:save={editorSave}
    on:force-save={editorForceSave}
    bind:isOverwrite
    bind:this={editor} />
</ConfigLayout>
