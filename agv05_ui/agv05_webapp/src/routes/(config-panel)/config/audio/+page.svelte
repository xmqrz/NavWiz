<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getToastStore } from '@skeletonlabs/skeleton';

  import inView from 'actions/in-view.js';
  import audio from '$lib/shared/services/config/audio';
  import Form from 'components/Form.svelte';
  import { scrollTop } from '$lib/utils';

  export let data;

  let addVisible = false;
  const toastStore = getToastStore();

  let layout = data.parameter.layout;
  let form;

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    save();
  }

  function save() {
    audio
      .update(form)
      .then((d) => {
        // TODO: hack for now since may return 202 when got warning.
        if (d.detail) {
          toastStore.trigger({
            background: 'variant-filled-warning',
            message: d.detail,
            timeout: 6000,
            hoverable: true
          });
          return;
        } else {
          toastStore.trigger({
            message: audio.successUpdateMsg(),
            timeout: 3000,
            hoverable: true
          });
          scrollTop();
        }
        // Prevent using preloaded data
        // Update value, validation/modification may occured on server side.
        Object.assign(data.parameter, d);
        // trigger update
        data = data;
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title={'Audio'} validation={false}>
  <form class="w-full lg:w-3/4" bind:this={form} on:submit|preventDefault novalidate>
    <Form initial={data.parameter} properties={data.properties} {layout} />
    <div
      class="mt-7 flex space-x-1 px-3"
      use:inView
      on:enter={() => (addVisible = true)}
      on:exit={() => (addVisible = false)}>
      <button
        type="submit"
        class="variant-filled-primary btn"
        on:click|preventDefault={onSubmit}>
        Submit
      </button>
    </div>
    {#if !addVisible}
      <div class="variant-glass-surface fixed bottom-0 left-0 flex w-full space-x-1 p-3 px-8">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}>
          Submit
        </button>
      </div>
    {/if}
  </form>
</ConfigLayout>
