<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import license from '$lib/shared/services/config/license';

  const toastStore = getToastStore();

  let confirm = '';
  let errors = {};

  function onSubmit() {
    errors = {};
    license
      .ret()
      .then((v) => {
        toastStore.trigger({
          message: license.successReturnMsg(),
          timeout: 3000,
          hoverable: true
        });
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
            })
            .finally(function () {
              toastStore.trigger({
                message: license.unilateralReturnMsg(),
                timeout: 3000,
                hoverable: true
              });
              toastStore.trigger({
                message: errors.detail,
                timeout: 3000,
                hoverable: true
              });
              goto('/config/help', { invalidateAll: true });
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
  <div class="p-3 text-lg font-semibold">
    Are you sure you want to return the license key? This will deactivate the license on this
    device.
  </div>
  <div class="p-3 px-10">
    <label class="label">
      <span>Type "Return License" in the textbox below to confirm.</span>
      <input name="confirm" class="input" type="text" bind:value={confirm} required />
    </label>
  </div>
  <div class="px-3 pt-7">
    <button
      class="variant-filled-primary btn"
      class:disabled={confirm !== 'Return License'}
      type="submit">Return License</button>
  </div>
</form>
