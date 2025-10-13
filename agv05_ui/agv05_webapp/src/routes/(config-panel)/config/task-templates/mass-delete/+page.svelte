<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let data;

  const toastStore = getToastStore();

  function onSubmit() {
    taskTemplates
      .massDelete(data.taskTemplates.map((tt) => tt.id))
      .then(() => {
        toastStore.trigger({
          message: taskTemplates.successMassDeleteMsg(data.taskTemplates.length),
          timeout: 3000,
          hoverable: true
        });
        // Prevent using preloaded data
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title="Delete multiple task templates"
  back={['Task Templates', taskTemplates.listUrl()]}>
  <div class="pt-3 lg:pl-20">
    Are you sure you want to delete the following task templates?
  </div>
  <div class="pt-3 lg:px-20">
    <h4 class="h4">Task Templates</h4>
    <div class="table-container max-h-96 pt-3">
      <table class="table table-hover">
        <thead>
          <tr>
            <th>#</th>
            <th>Name</th>
          </tr>
        </thead>
        <tbody>
          {#each data.taskTemplates as tt, i}
            <tr>
              <td>{i + 1}</td>
              <td>
                {tt.name}
                {#if tt.category}
                  <span class="variant-filled-surface chip float-right">{tt.category}</span>
                {/if}
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  </div>
  <div class="pt-4 opacity-50 lg:pl-20">
    {data.taskTemplates.length} task templates selected
  </div>
  <div class="pt-3 lg:px-20">
    <div class="space-y-5 p-3">
      <button
        type="submit"
        class="variant-filled-primary btn"
        on:click|preventDefault={onSubmit}>
        Delete
      </button>
      <a href={taskTemplates.listUrl()} class="variant-filled-surface btn">Cancel</a>
    </div>
  </div>
</ConfigLayout>
