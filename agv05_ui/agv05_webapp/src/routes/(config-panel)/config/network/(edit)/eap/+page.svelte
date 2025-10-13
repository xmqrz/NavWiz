<script>
  import { onMount } from 'svelte';
  import { getToastStore } from '@skeletonlabs/skeleton';

  import { getAPI, formAPI } from '$lib/utils';
  import ClearableFileInput from 'components/ClearableFileInput.svelte';

  const pathname = '/config/network/eap';
  const toastStore = getToastStore();

  let caFileInput;
  let wifiFileInput;

  let data = {
    initial: {
      ca_crt: null,
      wifi_crt: null,
      wifi_crt_pass: null,
      error: {}
    },
    bind: {
      ca_crt: null,
      wifi_crt: null,
      wifi_crt_pass: ''
    }
  };

  const updateData = (init, d) => {
    if (Object.keys(d.error).length) {
      data.initial.error = d.error;
    } else {
      caFileInput.clearInput();
      wifiFileInput.clearInput();
      data.initial = d;
      data.bind = {
        ca_crt: data.initial.ca_crt
          ? API_URL + pathname + '?download=' + data.initial.ca_crt
          : undefined,
        wifi_crt: data.initial.wifi_crt
          ? API_URL + pathname + '?download=' + data.initial.wifi_crt
          : undefined,
        wifi_crt_pass: ''
      };
      if (!init) {
        toastStore.trigger({
          message: 'EAP settings updated.',
          timeout: 3000,
          hoverable: true
        });
      }
    }
  };

  onMount(async () => {
    updateData(true, await getAPI(pathname));
  });

  async function handleSubmitProxy(event) {
    updateData(false, await formAPI(pathname, event.target));
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
</script>

<form
  novalidate
  class="w-full lg:w-3/4"
  enctype="multipart/form-data"
  on:submit|preventDefault={handleSubmitProxy}>
  <fieldset class="space-y-4 rounded p-3 px-10">
    <div class="grid grid-cols-4">
      <label
        for="ca-crt"
        class="{formLabelClass} {'ca_crt' in data.initial.error ? 'text-error-600' : ''}">
        CA certificate
      </label>
      <div class="col-span-3">
        <ClearableFileInput
          id="ca-crt"
          accept=".crt,.pem"
          name="ca_crt"
          bind:this={caFileInput}
          bind:value={data.bind.ca_crt}
          bind:filename={data.initial.ca_crt} />
        {#if 'ca_crt' in data.initial.error}
          <strong class="text-error-600">{data.initial.error.ca_crt}</strong>
        {/if}
      </div>
    </div>

    <div class="grid grid-cols-4">
      <label
        for="crt"
        class="{formLabelClass} {'wifi_crt' in data.initial.error ? 'text-error-600' : ''}">
        Wi-Fi certificate
      </label>
      <div class="col-span-3">
        <ClearableFileInput
          id="crt"
          class="input {'wifi_crt' in data.initial.error ? 'input-error' : ''}"
          accept=".p12,.pfx"
          name="wifi_crt"
          bind:this={wifiFileInput}
          bind:value={data.bind.wifi_crt}
          bind:filename={data.initial.wifi_crt} />
        {#if 'wifi_crt' in data.initial.error}
          <strong class="text-error-600">{data.initial.error.wifi_crt}</strong>
        {/if}
      </div>
    </div>

    <div class="grid grid-cols-4">
      <label
        for="crt-pass"
        class="{formLabelClass} {'wifi_crt_pass' in data.initial.error
          ? 'text-error-600'
          : ''}">
        Wi-Fi certificate password
      </label>
      <div class="col-span-3">
        <input
          id="crt-pass"
          class="input {'wifi_crt_pass' in data.initial.error ? 'input-error' : ''}"
          type="password"
          name="wifi_crt_pass"
          placeholder={data.initial.wifi_crt_pass}
          bind:value={data.bind.wifi_crt_pass} />
        {#if 'wifi_crt_pass' in data.initial.error}
          <strong class="text-error-600">{data.initial.error.wifi_crt_pass}</strong>
        {/if}
      </div>
    </div>
  </fieldset>
  <div class="mt-7 grid grid-cols-4 px-10">
    <div class="col-span-3 col-start-2">
      <button type="submit" class="variant-filled-primary btn">Submit</button>
    </div>
  </div>
</form>
