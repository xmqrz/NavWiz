<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Form from 'components/Form.svelte';
  import inView from 'actions/in-view.js';
  import permissions from '$lib/shared/services/config/permissions';
  import { scrollTop } from '$lib/utils';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let addVisible = false;
  let form;
  let initial = data.permissions;
  let layout = data.permissions.layout;
  let properties = data.properties;

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    permissions
      .update(form)
      .then((d) => {
        toastStore.trigger({
          message: permissions.successUpdateMsg(),
          timeout: 3000,
          hoverable: true
        });
        // Update value, validation/modification may occured on server side.
        Object.assign(data.permissions, d);
        // trigger update
        initial = data.permissions;
        scrollTop();
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function onFactoryReset() {
    modalStore.trigger({
      type: 'confirm',
      title: 'Reset Permissions',
      body: 'Are you sure you want to reset all permissions to factory default settings?',
      response: (r) => {
        if (!r) {
          return;
        }
        factorReset();
      }
    });
  }

  function factorReset() {
    permissions
      .reset()
      .then((d) => {
        toastStore.trigger({
          message: permissions.successResetMsg(),
          timeout: 3000,
          hoverable: true
        });
        // Update value, validation/modification may occured on server side.
        Object.assign(data.permissions, d);
        // trigger update
        initial = data.permissions;
        scrollTop();
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title="Permissions" validation={false}>
  <form bind:this={form} on:submit|preventDefault novalidate>
    <Form {properties} {initial} {layout} />
    <div class="mt-7 grid grid-cols-4">
      <div
        class="col-span-3 col-start-2 flex space-x-1 px-3"
        use:inView
        on:enter={() => (addVisible = true)}
        on:exit={() => (addVisible = false)}>
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Update
        </button>
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onFactoryReset}>
          Reset to Factory Default
        </button>
      </div>
    </div>
    {#if !addVisible}
      <div
        class="variant-glass-surface fixed bottom-0 left-0 grid w-full grid-cols-4 p-3 px-8">
        <div class="col-span-3 col-start-2 flex space-x-1">
          <button
            type="submit"
            class="variant-filled-primary btn"
            on:click|preventDefault={onSubmit}>
            Update
          </button>
          <button
            type="submit"
            class="variant-filled-primary btn"
            on:click|preventDefault={onFactoryReset}>
            Reset to Factory Default
          </button>
        </div>
      </div>
    {/if}
  </form>
</ConfigLayout>
