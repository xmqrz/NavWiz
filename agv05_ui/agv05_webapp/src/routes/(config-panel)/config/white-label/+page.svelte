<script>
  import { getToastStore } from '@skeletonlabs/skeleton';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Form from 'components/Form.svelte';
  import whiteLabel from '$lib/shared/services/config/white-label';

  export let data;

  const toastStore = getToastStore();

  let form;
  let initial = data.whiteLabel;
  const layout = {
    parameters: ['address_label', 'copyright_label', 'favicon_label', 'logo_label'],
    groups: {}
  };
  let properties = data.properties;

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    whiteLabel
      .update(form)
      .then((d) => {
        toastStore.trigger({
          message: whiteLabel.successUpdateMsg(),
          timeout: 3000,
          hoverable: true
        });
        // Update value, validation/modification may occured on server side.
        Object.assign(data.whiteLabel, d);
        // trigger update
        initial = data.whiteLabel;
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title="Edit White Label Settings" validation={false}>
  <form bind:this={form} on:submit|preventDefault novalidate>
    <Form {properties} {initial} {layout} />
    <div class="mt-7 grid grid-cols-4 space-x-1 px-10">
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
