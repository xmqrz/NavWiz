<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import TaskTemplateEditor from '../../add/TaskTemplateEditor.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';
  export let data;
</script>

<ConfigLayout
  title={`View cached task template "${data.taskTemplate.name}"`}
  back={['Task Templates', taskTemplates.listUrl()]}>
  <TaskTemplateEditor {data} editable={false} preserve={true} />
</ConfigLayout>
