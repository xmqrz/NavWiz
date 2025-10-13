<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import license from '$lib/shared/services/config/license';

  export let req;

  const toastStore = getToastStore();

  const a_placeholder = 'blob:navwiz.lrq';

  let files = [];
  let fileInput;
  let errors = {};

  function download(evt) {
    let a = evt.target;

    let blob = new Blob([req.license_req], { type: 'octet/stream' });
    let url = window.URL.createObjectURL(blob);

    a.href = url;

    setTimeout(() => {
      a.href = a_placeholder;
      window.URL.revokeObjectURL(url);
    }, 150);
  }

  function onSubmit() {
    errors = {};
    files[0]
      .text()
      .then(save)
      .catch((e) => {
        errors = { detail: 'Invalid license key.' };
      });
    fileInput.value = '';
  }

  function save(key) {
    let payload = {
      license_key: key
    };
    license
      .offline(payload)
      .then((r) => {
        toastStore.trigger({
          message: license.successUpdateMsg(),
          timeout: 3000,
          hoverable: true
        });
        if (r.days && !(r.features & 1)) {
          toastStore.trigger({
            message: `You have activated a trial license. You have ${r.days} day(s) remaining to evaluate this product.`,
            timeout: 3000,
            hoverable: true
          });
        }
        goto('/config/help', { invalidateAll: true });
      })
      .catch((e) => {
        if (e.errorCode === 400) {
          const response = e.cause;
          response
            .json()
            .then(function (err) {
              errors = err;
            })
            .catch(function () {
              errors = { detail: e.message };
            });
          toastStore.trigger({
            message: 'Check form for errors!',
            timeout: 3000,
            hoverable: true
          });
          return;
        }
        errors = { detail: e.message };
        toastStore.trigger({
          message: errors.detail,
          timeout: 3000,
          hoverable: true
        });
      });
  }
</script>

<form on:submit|preventDefault={onSubmit}>
  {#if errors.detail}
    <div class="alert variant-filled-error">
      <div class="alert-message">{errors.detail}</div>
    </div>
  {/if}
  <div class="p-3 text-lg font-semibold">Please upload the license key file.</div>
  <div class="p-3 px-10">
    <div>
      <span class="text-lg font-semibold">License Request</span>
      <div class="pl-2">
        <a class="anchor" href={a_placeholder} download="navwiz.lrq" on:click={download}>
          <i class="fa fa-download"></i> Download
        </a>
      </div>
    </div>
  </div>
  <div class="p-3 px-10">
    <label class="label" class:text-error-500={errors.license_key}>
      <span class="text-lg font-semibold">License Key</span>
      <input
        name="license_key"
        class="input"
        type="file"
        accept=".lic"
        bind:files
        bind:this={fileInput}
        required />
      {#each errors.license_key || [] as e}
        <br />
        <div>{e}</div>
      {/each}
    </label>
  </div>
  <div class="px-3 pt-7">
    <button class="variant-filled-primary btn" type="submit">Submit</button>
  </div>
</form>
