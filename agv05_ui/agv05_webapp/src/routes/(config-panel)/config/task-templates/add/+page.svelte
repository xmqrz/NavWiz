<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import TaskTemplateEditor from './TaskTemplateEditor.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let data;
  let editor;

  const toastStore = getToastStore();

  function editorSave(e) {
    // TODO: how to make this similar to successmixin.
    taskTemplates
      .add(e.detail)
      .then((d) => {
        toastStore.trigger({
          message: taskTemplates.successAddMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        editor.muteUnload();
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        if (editor.saveErrorCB(e)) {
          return;
        }
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title={`Add new task template`}
  back={['Task Templates', taskTemplates.listUrl()]}>
  <TaskTemplateEditor {data} on:save={editorSave} bind:this={editor} />
</ConfigLayout>
