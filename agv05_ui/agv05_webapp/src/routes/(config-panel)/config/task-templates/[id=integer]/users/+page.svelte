<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let data;
  const toastStore = getToastStore();

  let allowedGroups = data.taskTemplate.allowed_groups;
  let allowedUsers = data.taskTemplate.allowed_users;
  let dirty = false;

  function onSubmit() {
    taskTemplates
      .updateUsers(data.taskTemplate.id, {
        allowed_groups: allowedGroups,
        allowed_users: allowedUsers
      })
      .then((d) => {
        toastStore.trigger({
          message: taskTemplates.successUpdateMsg(d.name),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function triggerDirty() {
    if (!dirty) {
      dirty = true;
    }
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
</script>

<ConfigLayout
  title={`Edit users of task template "${data.taskTemplate.name}"`}
  back={['Task Templates', taskTemplates.listUrl()]}>
  <form action="" class="w-full lg:w-3/4">
    <div>
      <label class="grid grid-cols-4 p-3 px-10">
        <span class={formLabelClass}>Name</span>
        <input class="input col-span-3" type="text" value={data.taskTemplate.name} disabled />
      </label>
      <div class="grid grid-cols-4 p-3 px-10">
        <span class={formLabelClass}>Allowed groups</span>
        <div class="col-span-3 space-y-2">
          {#each data.taskTemplate.available_groups as group}
            <label class="flex items-center space-x-2">
              <input
                class="checkbox"
                type="checkbox"
                value={group[0]}
                bind:group={allowedGroups}
                on:change={triggerDirty} />
              <span>{group[1]}</span>
            </label>
          {/each}
        </div>
      </div>
      <div class="grid grid-cols-4 p-3 px-10">
        <span class={formLabelClass}>Allowed users</span>
        <div class="col-span-3 space-y-2">
          {#each data.taskTemplate.available_users as user}
            <label class="flex items-center space-x-2">
              <input
                class="checkbox"
                type="checkbox"
                value={user[0]}
                bind:group={allowedUsers}
                on:change={triggerDirty} />
              <p>{user[1]}</p>
            </label>
          {/each}
        </div>
      </div>
      <div class="mt-7 grid grid-cols-4 px-10">
        <div class="col-span-3 col-start-2">
          <button
            type="submit"
            class="variant-filled-primary btn"
            on:click|preventDefault={onSubmit}>
            Submit
          </button>
        </div>
      </div>
    </div>
  </form>
</ConfigLayout>
