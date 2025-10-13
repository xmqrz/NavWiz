<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import license from '$lib/shared/services/config/license';

  const toastStore = getToastStore();

  let formData = {};
  let errors = {};

  function onSubmit() {
    errors = {};
    license
      .online(formData)
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
    formData = {};
  }
</script>

<form on:submit|preventDefault={onSubmit}>
  {#if errors.detail}
    <div class="alert variant-filled-error">
      <div class="alert-message">{errors.detail}</div>
    </div>
  {/if}
  <div class="p-3 text-lg font-semibold">Please enter the license activation key.</div>
  <div class="p-3 px-10">
    <label class="label" class:text-error-500={errors.activation_key || errors.fingerprint}>
      <span class="text-lg font-semibold">Activation Key</span>
      <input
        name="activation_key"
        class="input"
        type="text"
        bind:value={formData.activation_key}
        required />
      {#each errors.activation_key || errors.fingerprint || [] as e}
        <br />
        <div>{e}</div>
      {/each}
    </label>
  </div>
  <div class="px-3 pt-7">
    <button class="variant-filled-primary btn" type="submit">Submit</button>
  </div>
</form>
