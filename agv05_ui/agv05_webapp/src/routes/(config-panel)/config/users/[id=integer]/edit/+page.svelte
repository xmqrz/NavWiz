<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import users from '$lib/shared/services/config/users';

  export let data;
  const toastStore = getToastStore();

  let form;
  let username = data.user.username;
  let user_group_id = data.user.groups[0] || undefined;
  let is_active = data.user.is_active;
  let dirty = false;
  let error = {};

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    users
      .update(data.user.id, {
        username: username,
        groups: [user_group_id],
        is_active: is_active
      })
      .then((d) => {
        toastStore.trigger({
          message: users.successUpdateMsg(d.username),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(users.listUrl()).then(() => invalidateAll());
      })
      .catch(async (e) => {
        if (e.errorCode === 400) {
          const d = await e.cause.json();
          if (d.username && Array.isArray(d.username)) {
            d.username = d.username[0];
          }
          if (d.groups && Array.isArray(d.groups)) {
            d.groups = d.groups[0];
          }
          error = d;
          return;
        }
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
  title={`Edit user "${data.user.username}"`}
  back={['Users', users.listUrl()]}
  validation={false}>
  <form bind:this={form} action="" class="w-full space-y-4 lg:w-3/4">
    <div class="grid grid-cols-4">
      <span class={formLabelClass} class:text-red-500={error.username}>Username</span>
      <input
        class="input col-span-3"
        class:input-error={error.username}
        type="text"
        required
        bind:value={username}
        on:change={triggerDirty} />
      {#if error.username}
        <span class="col-span-3 col-start-2 text-red-500">{error.username}</span>
      {/if}
    </div>
    <div class="grid grid-cols-4">
      <span class={formLabelClass} class:text-red-500={error.groups}>Permission group</span>
      <div class="col-span-3 space-y-2">
        <select
          required
          class="select rounded-token"
          class:input-error={error.groups}
          bind:value={user_group_id}
          on:change={triggerDirty}>
          {#each data.availableGroups as group}
            <option value={group.id}>{group.name}</option>
          {/each}
        </select>
      </div>
      {#if error.groups}
        <span class="col-span-3 col-start-2 text-red-500">{error.groups}</span>
      {/if}
    </div>
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <input
          class="checkbox"
          type="checkbox"
          bind:checked={is_active}
          on:change={triggerDirty} />
        <span class={formLabelClass}>Active</span>
        <br />
        <span class="text-gray-500"
          >Designates whether this user should be treated as active. Unselect this instead of
          deleting accounts.</span>
      </div>
    </div>
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Submit
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
