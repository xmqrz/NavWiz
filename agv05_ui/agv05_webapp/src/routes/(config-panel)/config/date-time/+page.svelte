<script>
  import { beforeNavigate, goto } from '$app/navigation';
  import { DateInput } from 'date-picker-svelte';
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';

  import datetimeService from '$lib/shared/services/config/datetime';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { toServerTZ } from 'stores/server-clock.js';

  export let data;

  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let form;
  let errors = {};
  let dirty = false;
  let date_and_time = toServerTZ(new Date(data.datetime.date_and_time));
  let initial_date_and_time = date_and_time;

  function triggerDirty() {
    dirty = true;
  }

  function stageData() {
    return {
      date_and_time: date_and_time,
      ntp_server: data.datetime.ntp_server,
      ntp_sync: data.datetime.ntp_sync,
      time_zone: data.datetime.time_zone
    };
  }

  function onSubmit() {
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    save();
  }

  $: if (initial_date_and_time !== date_and_time) {
    initial_date_and_time = date_and_time;
    triggerDirty();
  }

  function save() {
    modalStore.trigger({
      type: 'confirm',
      title: 'Please Confirm',
      body: `Are you sure you wish to update date time configuration and soft reboot?`,
      response: (r) => {
        if (r) {
          datetimeService
            .update(stageData())
            .then(() => {
              toastStore.trigger({
                message: datetimeService.successUpdateMsg(),
                timeout: 3000,
                hoverable: true
              });
              dirty = false;
              goto('/', { invalidateAll: true });
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
      }
    });
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
</script>

<ConfigLayout title="Date and Time Configuration" validation={false}>
  {#if errors.detail}
    <div class="alert variant-filled-error">
      <div class="alert-message">{errors.detail}</div>
    </div>
  {/if}
  <form bind:this={form} on:submit|preventDefault class="w-full space-y-2 lg:w-3/4">
    <label class="grid grid-cols-4 p-3" class:text-error-500={errors.date_and_time}>
      <span class={formLabelClass}>Date and Time</span>
      <div class="col-span-3">
        <DateInput bind:value={date_and_time} timePrecision="minute" />
        {#each errors.date_and_time || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4 p-3" class:text-error-500={errors.time_zone}>
      <span class={formLabelClass}>Time Zone</span>
      <div class="col-span-3">
        <select
          class="select rounded-token"
          name="time_zone"
          bind:value={data.datetime.time_zone}
          on:change={triggerDirty}
          required>
          {#each data.datetime.time_zone_choices as tz}
            <option value={tz[0]}>
              {tz[1]}
            </option>
          {/each}
        </select>
        {#each errors.time_zone || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4 p-3" class:text-error-500={errors.ntp_sync}>
      <div class="col-span-3 col-start-2">
        <input
          class="checkbox"
          type="checkbox"
          name="ntp_sync"
          bind:checked={data.datetime.ntp_sync}
          on:change={triggerDirty} />
        <span class={formLabelClass}>Auto synchronize with network time server</span>
        {#each errors.ntp_sync || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <label class="grid grid-cols-4 p-3" class:text-error-500={errors.ntp_server}>
      <span class={formLabelClass}>Local NTP Server</span>
      <div class="col-span-3">
        <input
          class="input"
          type="text"
          name="ntp_server"
          bind:value={data.datetime.ntp_server}
          on:change={triggerDirty} />
        <br />
        <span class="col-span-3 text-gray-500">
          The IP address of a preferred network time server for synchronization (optional).
        </span>
        {#each errors.ntp_server || [] as e}
          <br />
          <div>{e}</div>
        {/each}
      </div>
    </label>
    <div class="grid grid-cols-4 p-3">
      <span class={formLabelClass}>Synchronization Status</span>
      <input
        class="input col-span-3"
        type="text"
        bind:value={data.datetime.ntp_status}
        disabled />
    </div>
    <div class="grid grid-cols-4 space-y-6 p-2">
      <div class="col-span-3 col-start-2 font-bold text-red-500">
        Warning: A soft reboot will be performed after updating the date and time. Please make
        sure the AGV is not running any task.
      </div>
      <div class="col-span-3 col-start-2">
        <button
          type="submit"
          class="variant-filled-primary btn col-start-2"
          disabled={!dirty}
          on:click|preventDefault={onSubmit}>
          Update and Soft Reboot
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>

<style>
  :root {
    --date-picker-background: #d2d6e3;
    --date-input-width: 100%;
  }
</style>
