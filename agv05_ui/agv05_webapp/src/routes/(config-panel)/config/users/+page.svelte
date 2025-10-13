<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';
  import { getContext } from 'svelte';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Perm from 'components/Perm.svelte';
  import users from '$lib/shared/services/config/users';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  const user = getContext('user');

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(`/config/users?page=${page}`);
  }

  function onDeleteCB(user) {
    const modal = {
      type: 'confirm',
      title: 'Delete User',
      body: `Are you sure you want to delete "${user.username}"?`,
      response: (r) => {
        if (!r) {
          return;
        }
        removeUser(user);
      }
    };
    modalStore.trigger(modal);
  }

  function removeUser(user) {
    users
      .remove(user.id)
      .then(() => {
        toastStore.trigger({
          message: users.successDeleteMsg(user.username),
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

<ConfigLayout title="User" validation={false}>
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
            <th>Username</th>
            <th>Permission Group</th>
            <th>Active</th>
            <th>Operation</th>
          </tr>
        </thead>
        <tbody>
          {#each data.users as edited_user, i}
            <tr>
              <td>{i + 1 + data.limit * (data.page - 1)}</td>
              <td>{edited_user.username}</td>
              <td>{edited_user.groups[0] || '-'}</td>
              <td
                ><i
                  class="fa {edited_user.is_active
                    ? 'fa-solid fa-check-circle m-auto text-xl text-green-500'
                    : 'fa-times-circle m-auto text-xl text-red-500'}"></i
                ></td>
              <td>
                {#if edited_user.username != $user.username}
                  <Perm perms="auth.change_user">
                    <a
                      href={users.editUrl(edited_user.id)}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="Edit"><i class="fa fa-pencil"></i></a>
                    <a
                      href={users.changePasswordUrl(edited_user.id)}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="Change Password"><i class="fa fa-key"></i></a>
                    <a
                      href={users.authTokenUrl(edited_user.id)}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="Auth Token"><i class="fa fa-ticket"></i></a>
                  </Perm>
                  <Perm perms="auth.delete_user">
                    <button
                      type="button"
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      on:click={() => onDeleteCB(edited_user)}
                      tip-title="Delete">
                      <i class="fa fa-times"></i></button>
                  </Perm>
                {:else if $user.login}
                  <Perm perms="system.change_own_password">
                    <a
                      href={users.changeOwnPasswordUrl(users.listUrl())}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="Change Own Password"><i class="fa fa-key"></i></a>
                    <a
                      href={users.myAuthTokenUrl(users.listUrl())}
                      class="variant-filled btn mb-1 w-7 rounded p-1"
                      tip-title="My Auth Token"><i class="fa fa-ticket"></i></a>
                  </Perm>
                {/if}
              </td>
            </tr>
          {:else}
            <tr>
              <td colspan="5">No users yet.</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
    <Perm perms="auth.add_user">
      <a href={users.addUrl()} class="variant-filled-secondary btn text-white">
        <i class="fa-solid fa-plus mr-2"></i>
        Add new user
      </a>
    </Perm>

    <Perm perms="auth.change_user">
      <div class="space-y-2 py-4">
        <span class="text-2xl"><i class="fa fa-caret-right mr-2"></i>AGV Panel</span>
        <div>
          <a
            href={users.panelPinUrl()}
            class="btn-default variant-filled-secondary btn text-white">
            <i class="fa fa-key mr-2"></i> Change Protection Pin</a>
        </div>
      </div>
    </Perm>
  </div>
</ConfigLayout>
