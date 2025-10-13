<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto, invalidateAll } from '$app/navigation';
  import { getToastStore } from '@skeletonlabs/skeleton';

  import inView from 'actions/in-view.js';
  import parameters from '$lib/shared/services/config/parameters';
  import Form from 'components/Form.svelte';

  export let data;

  let addVisible = false;
  const toastStore = getToastStore();

  let layout = data.parameter.layout;
  let form;

  function onSubmit() {
    save();
  }

  function onSubmitAndEdit() {
    save(false);
  }

  function save(goBack = true) {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    parameters
      .update(data.parameter.key, data.allowProtected, form)
      .then((d) => {
        // TODO: hack for now since may return 202 when got warning.
        let error = false;
        if (d.detail) {
          error = true;
          toastStore.trigger({
            background: 'variant-filled-warning',
            message: d.detail,
            timeout: 6000,
            hoverable: true
          });
        } else {
          toastStore.trigger({
            message: parameters.successUpdateMsg(data.parameter.key),
            timeout: 3000,
            hoverable: true
          });
        }
        // Prevent using preloaded data
        if (goBack) {
          goto(parameters.listUrl()).then(() => invalidateAll());
        } else {
          if (error) {
            return;
          }
          // Update value, validation/modification may occured on server side.
          Object.assign(data.parameter, d);
          // trigger update
          data = data;
        }
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title={`Edit parameter component "${data.parameter.key}"`}
  back={['Parameters', parameters.listUrl()]}
  validation={false}>
  <form bind:this={form} on:submit|preventDefault novalidate>
    <Form initial={data.parameter} properties={data.properties} {layout} />
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
          Submit
        </button>
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmitAndEdit}>
          Submit and continue editing
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
            Submit
          </button>
          <button
            type="submit"
            class="variant-filled-primary btn"
            on:click|preventDefault={onSubmitAndEdit}>
            Submit and continue editing
          </button>
        </div>
      </div>
    {/if}
  </form>
</ConfigLayout>
