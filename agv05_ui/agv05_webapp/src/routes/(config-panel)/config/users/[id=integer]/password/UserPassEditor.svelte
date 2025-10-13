<script>
  import { beforeNavigate, afterNavigate } from '$app/navigation';
  import { createEventDispatcher } from 'svelte';

  export let data;
  const dispatch = createEventDispatcher();

  let dirty = false;
  let form;
  let password1;
  let password2;
  let validationMessage = '';

  afterNavigate(() => {
    password1 = undefined;
    password2 = undefined;
    validationMessage = '';
    dirty = false;
  });

  export function muteUnload() {
    dirty = false;
  }

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
    const personalInfo = [data.user.username];
    for (const info of personalInfo) {
      if (password1.toLowerCase().includes(info.toLowerCase())) {
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
    if (commonPasswords.includes(password1.toLowerCase())) {
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
      return;
    }
    dispatch('save', {
      password: password1
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

<form bind:this={form} action="">
  <div class="grid">
    <div class="grid grid-cols-4 p-3">
      <span class="font-medium">New password</span>
      <input
        class="input col-span-3"
        type="password"
        required
        bind:value={password1}
        on:change={triggerDirty} />
    </div>
    <div class="grid grid-cols-4 p-3">
      <span class="font-medium"></span>
      <div class="col-span-3 pl-5">
        <ul class="list-disc">
          <li>Your password can't be too similar to your other personal information.</li>
          <li>Your password must contain at least 8 characters.</li>
          <li>Your password can't be a commonly used password.</li>
          <li>Your password can't be entirely numeric.</li>
        </ul>
      </div>
    </div>
    <div class="grid grid-cols-4 p-3">
      <span class="font-medium">New password confirmation</span>
      <input
        class="input col-span-3"
        type="password"
        required
        bind:value={password2}
        on:change={triggerDirty} />
    </div>
    {#if validationMessage}
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium"></span>
        <div class="col-span-3">
          <span class="text-red-500">{validationMessage}</span>
        </div>
      </div>
    {/if}
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Update
        </button>
      </div>
    </div>
  </div>
</form>
