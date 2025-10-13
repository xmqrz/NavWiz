<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';
  import * as _ from 'lodash-es';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let data;

  const toastStore = getToastStore();

  let usersAssoc = _.fromPairs(data.availableUsers);
  let groupAssoc = _.fromPairs(data.availableGroups);
  let allowedUsers = [];
  let allowedGroups = [];
  let dirty = false;

  if (
    data.taskTemplates.every((tt) =>
      isArrayEqual(tt.allowed_users, data.taskTemplates[0].allowed_users)
    )
  ) {
    allowedUsers = data.taskTemplates[0].allowed_users;
  }
  if (
    data.taskTemplates.every((tt) =>
      isArrayEqual(tt.allowed_groups, data.taskTemplates[0].allowed_groups)
    )
  ) {
    allowedGroups = data.taskTemplates[0].allowed_groups;
  }

  function isArrayEqual(arr1, arr2) {
    if (arr1.length !== arr2.length) {
      return false;
    }

    return arr1.every((item, index) => item === arr2[index]);
  }

  function triggerDirty() {
    dirty = true;
  }

  function onSubmit() {
    if (!dirty) {
      return;
    }

    let payload = {
      allowed_users: allowedUsers,
      allowed_groups: allowedGroups
    };
    taskTemplates
      .massUsersUpdate(
        data.taskTemplates.map((tt) => tt.id),
        payload
      )
      .then(() => {
        toastStore.trigger({
          message: taskTemplates.successMassUpdateMsg(data.taskTemplates.length),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
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
  title="Multiple task templates category update"
  back={['Task Templates', taskTemplates.listUrl()]}>
  <div class="pt-3 lg:px-20">
    <h4 class="h4">Task Templates</h4>
    <div class="table-container max-h-96 pt-3">
      <table class="table table-hover">
        <thead>
          <tr>
            <th>#</th>
            <th>Name</th>
            <th>Allowed Groups</th>
            <th>Allowed Users</th>
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
              <td>
                {tt.allowed_groups && tt.allowed_groups.length
                  ? tt.allowed_groups.map((i) => groupAssoc[i]).join(', ')
                  : '-'}
              </td>
              <td>
                {tt.allowed_users && tt.allowed_users.length
                  ? tt.allowed_users.map((i) => usersAssoc[i]).join(', ')
                  : '-'}
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
      <div class="p-3">
        <span class="font-medium">Allowed groups</span>
        <div class="space-y-2">
          {#each data.availableGroups as group}
            <label class="flex items-center space-x-2">
              <input
                class="checkbox"
                type="checkbox"
                value={group[0]}
                bind:group={allowedGroups}
                on:change={triggerDirty} />
              <p>{group[1]}</p>
            </label>
          {/each}
        </div>
      </div>
      <div class="p-3">
        <span class="font-medium">Allowed users</span>
        <div class="space-y-2">
          {#each data.availableUsers as user}
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
      <button
        type="submit"
        class="variant-filled-primary btn"
        on:click|preventDefault={onSubmit}
        disabled={!dirty}>
        Submit
      </button>
    </div>
  </div>
</ConfigLayout>
