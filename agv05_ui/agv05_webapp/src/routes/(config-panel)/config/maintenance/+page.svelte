<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { invalidateAll } from '$app/navigation';
  import * as _ from 'lodash-es';

  import Perm from 'components/Perm.svelte';
  import maint from '$lib/shared/services/config/maintenance';

  export let data;

  const toastStore = getToastStore();

  let editing = false;
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
    let payload = formData;
    if (payload.manufacture_date === '') {
      delete payload.manufacture_date;
    }

    errors = {};
    maint
      .update(payload)
      .then((v) => {
        toastStore.trigger({
          message: maint.successUpdateMsg(),
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

  let uploadInput;

  async function upload() {
    let data;
    try {
      data = JSON.parse(await uploadInput.files[0].text());
    } catch {
      toastStore.trigger({
        message: 'Invalid data file.',
        timeout: 3000,
        hoverable: true
      });
      return;
    }

    try {
      await maint.update(data);
    } catch (e) {
      let errors = { detail: e.message };
      if (e.errorCode === 400) {
        const response = e.cause;
        try {
          data = await response.json();
          if (data.maker_key || data.maker_key_2) {
            errors.detail = 'Data file is either expired or not intended for this machine.';
          } else {
            errors.detail = 'Invalid data file.';
          }
        } catch {}
      }
      toastStore.trigger({
        message: errors.detail,
        timeout: 3000,
        hoverable: true
      });
      return;
    }

    toastStore.trigger({
      message: maint.successUploadMsg(),
      timeout: 3000,
      hoverable: true
    });
    invalidateAll();
  }

  async function populate() {
    try {
      await maint.sync();
    } catch (e) {
      let errors = { detail: e.message };
      if (e.errorCode === 400) {
        const response = e.cause;
        try {
          errors = await response.json();
        } catch {}
      }
      toastStore.trigger({
        message: errors.detail,
        timeout: 3000,
        hoverable: true
      });
      return;
    }

    toastStore.trigger({
      message: maint.successSyncMsg(),
      timeout: 3000,
      hoverable: true
    });
    invalidateAll();
  }

  const labelClass = 'text-lg font-semibold pr-4 text-right';
</script>

{#if !editing}
  <div class="my-4 w-full space-y-4 px-10 lg:w-3/4">
    <Perm perms="system.change_assembly_info">
      <div class="grid grid-cols-4">
        <div class="col-span-3 col-start-2">
          <button type="button" class="variant-filled-secondary btn" on:click={edit}>
            Edit
          </button>
          <button
            type="button"
            class="variant-filled-secondary btn"
            on:click={() => uploadInput.click()}>
            Upload
          </button>
          <input
            name="upload"
            type="file"
            accept=".json"
            hidden
            bind:this={uploadInput}
            on:change={upload} />
          <button
            type="button"
            class="btn variant-gradient-secondary-tertiary bg-gradient-to-br"
            on:click={populate}>
            Populate from DF Hub
          </button>
        </div>
      </div>
    </Perm>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>AGV Model</span>
      <div class="col-span-3">{data.data.agv_model ?? '-'}</div>
    </div>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Serial Number </span>
      <div class="col-span-3">{data.data.serial_number ?? '-'}</div>
    </div>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Manufacture Date</span>
      <div class="col-span-3">{data.data.manufacture_date ?? '-'}</div>
    </div>
    <div class="grid grid-cols-4 items-center">
      <span class={labelClass}>Mileage (m)</span>
      <div class="col-span-3">{data.data.mileage}</div>
    </div>
  </div>
  <div class="mt-6">
    <span class={labelClass}>Parts</span>
  </div>
  <div class="table-container my-4">
    <table class="table table-hover">
      <thead>
        <tr>
          <th>No.</th>
          <th>Module</th>
          <th>Part</th>
          <th>Serial Number</th>
          <th>Remarks</th>
          <th>Maintenance &amp; Warranty</th>
          <th>Status</th>
        </tr>
      </thead>
      <tbody>
        {#each data.data.parts as part, i}
          {@const mw = part.maintenance}
          <tr>
            <td>{i + 1}</td>
            <td>{part.module}</td>
            <td>{part.part}</td>
            <td>{part.serial}</td>
            <td class="!whitespace-pre-line">{part.remarks}</td>
            <td class="[&>div]:text-nowrap">
              {#each ['manufacturing_order_number', 'manufacture_date', 'delivery_date', 'installation_date', 'installation_mileage', 'last_service_date', 'last_service_mileage', 'next_service_date', 'next_service_mileage'] as k}
                {#if mw[k]}
                  <div>
                    <span class="text-xs font-bold">{_.startCase(k)}:</span>
                    {#if k.endsWith('_date')}
                      {mw[k].substring(0, 10)}
                    {:else if k.endsWith('mileage')}
                      {mw[k]}m
                    {:else}
                      {mw[k]}
                    {/if}
                  </div>
                {/if}
              {/each}
              {#if mw.has_warranty}
                {#each ['warranty_start_date', 'warranty_end_date'] as k}
                  {#if mw[k]}
                    <div>
                      <span class="text-xs font-bold">{_.startCase(k)}:</span>
                      {mw[k].substring(0, 10)}
                    </div>
                  {/if}
                {/each}
                {#if mw.warranty_months}
                  <div>
                    <span class="text-xs font-bold">Warranty period:</span>
                    {mw.warranty_months} months
                  </div>
                {/if}
                {#each ['warranty_mileage', 'warranty_start_mileage', 'warranty_end_mileage'] as k}
                  {#if mw[k]}
                    <div>
                      <span class="text-xs font-bold">{_.startCase(k)}:</span>
                      {mw[k]}m
                    </div>
                  {/if}
                {/each}
              {:else}
                <div>
                  <span class="text-xs font-bold">Warranty:</span>
                  N.A
                </div>
              {/if}
            </td>
            <td>{_.startCase(part.status)}</td>
          </tr>
        {:else}
          <tr>
            <td colspan="7">No parts.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
{:else}
  <form on:submit|preventDefault={onSubmit} class="my-4 w-full space-y-4 px-10 lg:w-3/4">
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
    <label class="grid grid-cols-4" class:text-error-500={errors.agv_model}>
      <span class={labelClass}>AGV Model</span>
      <div class="col-span-3">
        <input
          name="agv_model"
          class="input"
          class:input-error={errors.agv_model}
          type="text"
          bind:value={formData.agv_model}
          required />
        {#each errors.agv_model || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.serial_number}>
      <span class={labelClass}>Serial Number</span>
      <div class="col-span-3">
        <input
          name="serial_number"
          class="input"
          class:input-error={errors.serial_number}
          type="text"
          bind:value={formData.serial_number}
          required />
        {#each errors.serial_number || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.manufacture_date}>
      <span class={labelClass}>Manufacture Date</span>
      <div class="col-span-3">
        <input
          name="manufacture_date"
          class="input"
          class:input-error={errors.manufacture_date}
          type="date"
          bind:value={formData.manufacture_date}
          required />
        {#each errors.manufacture_date || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.overwrite_mileage}>
      <div class="col-span-3 col-start-2">
        <input
          name="overwrite_mileage"
          class="checkbox mr-1"
          class:input-error={errors.overwrite_mileage}
          type="checkbox"
          bind:checked={formData.overwrite_mileage} />
        <span class={labelClass}>Overwrite mileage</span>
        {#each errors.overwrite_mileage || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4" class:text-error-500={errors.mileage}>
      <span class={labelClass}>Mileage (m)</span>
      <div class="col-span-3">
        <input
          name="mileage"
          class="input"
          class:input-error={errors.mileage}
          type="number"
          bind:value={formData.mileage}
          min="0"
          disabled={!formData.overwrite_mileage}
          required={formData.overwrite_mileage} />
        {#each errors.mileage || [] as e}
          <br />
          <div>{e}</div>
        {/each}
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
    <label class="grid grid-cols-4" class:text-error-500={errors.maker_key_2}>
      <span class={labelClass}>Maker Key 2</span>
      <div class="col-span-3">
        <input
          name="maker_key_2"
          class="input"
          class:input-error={errors.maker_key_2}
          type="text"
          bind:value={formData.maker_key_2}
          maxlength="9"
          required />
        {#each errors.maker_key_2 || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
  </form>
{/if}
