<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { invalidateAll } from '$app/navigation';
  import * as _ from 'lodash-es';

  import Perm from 'components/Perm.svelte';
  import maint from '$lib/shared/services/config/maintenance';

  export let data;

  const toastStore = getToastStore();

  let editing = false;
  let form;
  let formData;
  let errors;

  function edit() {
    editing = true;
    formData = _.cloneDeep(data.data);
    errors = {};
  }

  function cancel() {
    editing = false;
  }

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }

    let payload = formData;
    if (payload.next_pm_due === '') {
      delete payload.next_pm_due;
    }

    errors = {};
    maint
      .updatePm(payload)
      .then((v) => {
        toastStore.trigger({
          message: maint.pmSuccessUpdateMsg(),
          timeout: 3000,
          hoverable: true
        });
        invalidateAll();
        cancel();
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
  }

  const labelClass = 'text-lg font-semibold pr-4 text-right';
</script>

{#if !editing}
  <div class="my-4 w-full space-y-4 px-10 lg:w-3/4">
    <Perm perms="system.change_preventive_maintenance">
      <div class="grid grid-cols-4">
        <div class="col-span-3 col-start-2">
          <button type="button" class="variant-filled-secondary btn" on:click={edit}>
            Edit
          </button>
        </div>
      </div>
    </Perm>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Next P.M. Due Date</span>
      <div class="col-span-3">{data.data.next_pm_due || '2020-08-01'}</div>
    </div>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Next P.M. Mileage (m)</span>
      <div class="col-span-3">{data.data.next_pm_mileage || 0}</div>
    </div>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Current Mileage (m)</span>
      <div class="col-span-3">{data.data.mileage}</div>
    </div>
  </div>
{:else}
  <form
    bind:this={form}
    on:submit|preventDefault={onSubmit}
    class="my-4 w-full space-y-4 px-10 lg:w-3/4">
    <div class="grid grid-cols-4">
      <div class="col-span-3 col-start-2">
        <button type="submit" class="variant-filled-secondary btn">Submit</button>
        <button type="button" class="variant-filled-surface btn" on:click={cancel}
          >Cancel</button>
      </div>
    </div>
    {#if errors.detail}
      <div class="alert variant-filled-error">
        <div class="alert-message">{errors.detail}</div>
      </div>
    {/if}
    <label class="grid grid-cols-4" class:text-error-500={errors.next_pm_due}>
      <span class={labelClass}>Next P.M. Due Date</span>
      <div class="col-span-3">
        <input
          name="next_pm_due"
          class="input"
          class:input-error={errors.next_pm_due}
          type="date"
          bind:value={formData.next_pm_due}
          required />
        {#each errors.next_pm_due || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.next_pm_mileage}>
      <span class={labelClass}>Next P.M. Mileage (m)</span>
      <div class="col-span-3">
        <input
          name="next_pm_mileage"
          class="input"
          class:input-error={errors.next_pm_mileage}
          type="number"
          bind:value={formData.next_pm_mileage}
          min="0"
          required />
        {#each errors.next_pm_mileage || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4">
      <span class={labelClass}>Current Mileage (m)</span>
      <div class="col-span-3">
        <input name="mileage" class="input" type="number" value={formData.mileage} disabled />
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.maker_key}>
      <span class={labelClass}>Maker Key</span>
      <div class="col-span-3">
        <input
          name="maker_key"
          class="input"
          class:input-error={errors.maker_key}
          type="text"
          bind:value={formData.maker_key}
          maxlength="9"
          required />
        {#each errors.maker_key || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
  </form>
{/if}
