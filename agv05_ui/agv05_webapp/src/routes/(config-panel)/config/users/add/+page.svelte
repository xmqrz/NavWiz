<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import users from '$lib/shared/services/config/users';

  export let data;
  const toastStore = getToastStore();

  let form;
  let username;
  let user_group_id;
  for (const item of data.availableGroups) {
    if (item.name === 'User') {
      user_group_id = item.id;
      break;
    }
  }
  let password1;
  let password2;
  let validationMessage = '';
  let dirty = false;
  let error = {};

  function validate() {
    if (!password1 || password1 !== password2) {
      password1 = null;
      password2 = null;
      validationMessage = "The two password fields didn't match";
      return false;
    }

    // Check if the password is at least 8 characters long
    if (password1.length < 8) {
      password1 = null;
      password2 = null;
      validationMessage = 'Your password must contain at least 8 characters.';
      return false;
    }

    // Check if the password is too similar to other personal information
    const personalInfo = [username];
    for (const info of personalInfo) {
      if (password1 && password1.toLowerCase().includes(info.toLowerCase())) {
        password1 = null;
        password2 = null;
        validationMessage =
          "Your password can't be too similar to your other personal information.";
        return false;
      }
    }

    // Check if the password contains numeric characters
    if (/^\d+$/.test(password1)) {
      password1 = null;
      password2 = null;
      validationMessage = "Your password can't be entirely numeric.";
      return false;
    }

    // Check if the password is a commonly used password
    const commonPasswords = ['password', 'password123', 'qwerty123', 'qwertyuiop'];
    if (password1 && commonPasswords.includes(password1.toLowerCase())) {
      password1 = null;
      password2 = null;
      validationMessage = "Your password can't be a commonly used password.";
      return false;
    }

    validationMessage = '';
    return true;
  }

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    if (!validate()) {
      error = {
        password: validationMessage
      };
      return;
    }
    users
      .add({
        username: username,
        groups: [user_group_id],
        password: password1,
        is_active: true
      })
      .then((d) => {
        toastStore.trigger({
          message: users.successAddMsg(d.username),
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
          if (d.password && Array.isArray(d.password)) {
            d.password = d.password[0];
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
</script>

<ConfigLayout title={`Add new user`} back={['Users', users.listUrl()]} validation={false}>
  <form bind:this={form} action="">
    <div class="grid">
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium" class:text-red-500={error.username}>Username</span>
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
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium" class:text-red-500={error.groups}>Permission group</span>
        <div class="col-span-3">
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
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium" class:text-red-500={error.password}>New password</span>
        <input
          class="input col-span-3"
          class:input-error={error.password}
          type="password"
          required
          bind:value={password1}
          on:change={triggerDirty} />
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium"></span>
        <div class="col-span-3 pl-5">
          <ul class="list-disc text-gray-500">
            <li>Your password can't be too similar to your other personal information.</li>
            <li>Your password must contain at least 8 characters.</li>
            <li>Your password can't be a commonly used password.</li>
            <li>Your password can't be entirely numeric.</li>
          </ul>
        </div>
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium" class:text-red-500={error.password}>
          New password confirmation
        </span>
        <input
          class="input col-span-3"
          class:input-error={error.password}
          type="password"
          required
          bind:value={password2}
          on:change={triggerDirty} />
        {#if error.password}
          <span class="col-span-3 col-start-2 text-red-500">{error.password}</span>
        {/if}
      </div>
      <div class="p-3">
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
