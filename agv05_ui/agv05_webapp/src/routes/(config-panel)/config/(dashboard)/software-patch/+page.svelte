<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto } from '$app/navigation';

  import patch from '$lib/shared/services/config/software-patch';

  export let data;

  let changelog = data.changelog;
  let form;
  let errors = {};

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    patch
      .update(form)
      .then((v) => {
        goto('/config/software-update-progress', {
          state: {
            message: patch.successUpdateMsg(),
            logFile: v.log_file
          }
        });
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
          return;
        }
        errors = { detail: e.message };
      });

    errors = {};
  }
</script>

<ConfigLayout title="Manage Patch" validation={false}>
  {#if errors.detail}
    <div class="alert variant-filled-error mb-3">
      <div class="alert-message">{errors.detail}</div>
    </div>
  {/if}
  <form bind:this={form} on:submit|preventDefault={onSubmit}>
    <div class="p-3 pt-0">
      <label class="label">
        <span class="text-lg font-semibold">Current changelog</span>
        <textarea class="textarea" rows="10" readonly>{changelog}</textarea>
      </label>
    </div>
    <div class="p-3">
      <label class="label">
        <span class="text-lg font-semibold">Patch file</span>
        <input name="patch_file" class="input" type="file" accept=".tar" required />
      </label>
      <div class="text-error-500">
        {#each errors.patch_file || [] as e}
          <div>{e}</div>
        {/each}
      </div>
    </div>
    <div class="px-3 pt-7">
      <button type="submit" class="variant-filled-primary btn">Apply Patch</button>
    </div>
  </form>
</ConfigLayout>
