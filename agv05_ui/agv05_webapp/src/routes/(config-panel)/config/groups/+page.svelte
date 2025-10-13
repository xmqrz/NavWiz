<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Perm from 'components/Perm.svelte';
  import groups from '$lib/shared/services/config/groups';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(`/config/groups?page=${page}`);
  }

  function onDeleteCB(group) {
    const modal = {
      type: 'confirm',
      title: 'Delete Group',
      body: `Are you sure you want to delete "${group.name}"?`,
      response: (r) => {
        if (!r) {
          return;
        }
        removeGroup(group);
      }
    };
    modalStore.trigger(modal);
  }

  function removeGroup(group) {
    groups
      .remove(group.id)
      .then(() => {
        toastStore.trigger({
          message: groups.successDeleteMsg(group.name),
          timeout: 3000,
          hoverable: true
        });
        invalidateAll();
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title="Groups" validation={false}>
  <div class="space-y-3">
    <div class="flex justify-end">
      <Paginator
        settings={{
          page: data.page - 1,
          limit: data.limit,
          size: data.count,
          amounts: []
        }}
        on:page={onPageChange} />
    </div>
    <div class="table-container">
      <table class="table table-hover">
        <thead>
          <tr>
            <th>#</th>
            <th>Group</th>
            <th>Operation</th>
          </tr>
        </thead>
        <tbody>
          {#each data.groups as obj, i}
            <tr>
              <td>{i + 1 + data.limit * (data.page - 1)}</td>
              {#if data.builtinGroups.includes(obj.name)}
                <td>
                  {obj.name}
                  <span
                    class="float-right rounded-lg bg-surface-400 p-1"
                    style="line-height:inherit">default</span>
                </td>
                <td></td>
              {:else}
                <td>{obj.name}</td>
                <td>
                  <Perm perms={['auth.change_group']}>
                    <a
                      href={groups.editUrl(obj.id)}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="Edit"><i class="fa fa-pencil"></i></a>
                  </Perm>
                  <Perm perms={['auth.delete_group']}>
                    <button
                      type="button"
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      on:click={() => onDeleteCB(obj)}
                      tip-title="Delete"><i class="fa fa-times"></i></button>
                  </Perm>
                </td>
              {/if}
            </tr>
          {:else}
            <tr><td colspan="3">No groups yet.</td></tr>
          {/each}
        </tbody>
      </table>
    </div>
    <div>
      <Perm perms={['auth.add_group']}>
        <a href={groups.addUrl()} class="variant-filled-secondary btn text-white">
          <i class="fa-solid fa-plus mr-2"></i>
          Add new group
        </a>
      </Perm>
    </div>
  </div>
</ConfigLayout>
