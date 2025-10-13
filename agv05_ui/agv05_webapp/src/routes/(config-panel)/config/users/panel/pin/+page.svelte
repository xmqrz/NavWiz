<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import users from '$lib/shared/services/config/users';

  let dirty = false;
  let form;
  let new_pin1;
  let new_pin2;
  let validationMessage = '';

  const toastStore = getToastStore();

  function validate() {
    if (!new_pin1 || new_pin1 !== new_pin2) {
      new_pin1 = null;
      new_pin2 = null;
      validationMessage = "The two pin fields didn't match";
      return false;
    }

    // Pin must consist of 6 digits.
    if (!/^\d{6}$/.test(new_pin1)) {
      new_pin1 = null;
      new_pin2 = null;
      validationMessage = 'Pin must consist of 6 digits.';
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
    users
      .changeProtectionPin({
        pin: new_pin1
      })
      .then(() => {
        toastStore.trigger({
          message: 'AGV Panel Protection Pin updated.',
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        // TODO: why goto invalidateAll option not working.
        goto(users.listUrl()).then(() => invalidateAll());
      })
      .catch((error) => {
        console.error('Error:', error);
        toastStore.trigger({
          message: 'Failed to change AGV Panel Protection Pin.',
          timeout: 3000,
          hoverable: true,
          type: 'error'
        });
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

<ConfigLayout
  title={`Change AGV panel protection pin`}
  back={['Users', users.listUrl()]}
  validation={false}>
  <form bind:this={form} action="">
    <div class="grid">
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">New pin</span>
        <input
          class="input col-span-3"
          type="password"
          required
          bind:value={new_pin1}
          on:change={triggerDirty} />
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">New pin confirmation</span>
        <input
          class="input col-span-3"
          type="password"
          required
          bind:value={new_pin2}
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
      <div class="p-3">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Update
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
