<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { getContext, onMount } from 'svelte';
  import { getAPI } from '$lib/utils';

  const pathname = '/config/network/https';
  const handleSubmit = getContext('networkSubmit');
  const modalStore = getModalStore();

  let data = {
    network_scheme: 'http',
    private_key: '',
    public_crt: '',
    gen_crt: false,
    error: {}
  };

  onMount(async () => {
    data = await getAPI(pathname);
  });

  async function handleSubmitProxy(event) {
    const { submitter: submitButton } = event;
    data.gen_crt = submitButton.value === 'gencrt';
    if (data.gen_crt) {
      modalStore.trigger({
        type: 'confirm',
        title: 'Generate Self Signed Certificate',
        body: 'Warning: this will overwrite current certificate.',
        response: (r) => {
          if (r) {
            handleSubmit(data);
          }
        }
      });
    } else {
      let res = await handleSubmit(data);
      data.error = res.error;
    }
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
</script>

<div class="container">
  <form class="w-full lg:w-3/4" on:submit|preventDefault={handleSubmitProxy}>
    <fieldset class="space-y-4 rounded p-3 px-10">
      <label class="grid grid-cols-4">
        <span
          class="{formLabelClass} {'network_scheme' in data.error ? 'text-error-600' : ''}">
          Network Scheme
        </span>
        <select
          required
          class="select {'network_scheme' in data.error
            ? 'input-error'
            : ''} col-span-3 rounded-token"
          bind:value={data.network_scheme}>
          <option value="http">HTTP</option>
          <option value="https">HTTPS</option>
        </select>
        <br />
        <span class="col-span-3 text-gray-500"
          >HTTPS scheme require public certificate bundle and private key.</span>
      </label>
      {#if 'network_scheme' in data.error}
        <div class="grid grid-cols-4">
          <span class="col-span-3 col-start-2 font-semibold text-error-600"
            >{data.error.network_scheme}</span>
        </div>
      {/if}

      <label class="grid grid-cols-4">
        <span class="{formLabelClass} {'private_key' in data.error ? 'text-error-600' : ''}">
          Private Key (KEY)
        </span>
        <textarea
          class="textarea font-mono {'private_key' in data.error
            ? 'input-error'
            : ''} col-span-3"
          rows="10"
          cols="40"
          bind:value={data.private_key} />
        <br />
        <span class="col-span-3 text-gray-500">Private key should not have a password.</span>
        {#if 'private_key' in data.error}
          <br />
          <span class="col-span-3 font-semibold text-error-600">{data.error.private_key}</span>
        {/if}
      </label>

      <label class="grid grid-cols-4">
        <span class="{formLabelClass} {'public_crt' in data.error ? 'text-error-600' : ''}">
          Certificate Bundle (CRT)
        </span>
        <textarea
          class="textarea font-mono {'public_crt' in data.error
            ? 'input-error'
            : ''} col-span-3"
          rows="10"
          cols="40"
          bind:value={data.public_crt} />
        {#if 'public_crt' in data.error}
          <br />
          <span class="col-span-3 font-semibold text-error-600">{data.error.public_crt}</span>
        {/if}
      </label>
    </fieldset>
    <div class="mt-7 grid grid-cols-4 px-10">
      <div class="col-span-2 col-start-2">
        <button type="submit" class="variant-filled-primary btn" value="submit">
          Submit
        </button>
        <button type="submit" class="variant-filled-primary btn" value="gencrt">
          Generate Certificate
        </button>
      </div>
    </div>
  </form>
</div>
