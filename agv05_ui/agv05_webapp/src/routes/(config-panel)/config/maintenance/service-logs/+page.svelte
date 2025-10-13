<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { invalidateAll } from '$app/navigation';
  import * as _ from 'lodash-es';

  import Perm from 'components/Perm.svelte';
  import maint from '$lib/shared/services/config/maintenance';

  export let data;

  const toastStore = getToastStore();

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
      await maint.updateServiceLogs(data);
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
      message: maint.slogSuccessUploadMsg(),
      timeout: 3000,
      hoverable: true
    });
    invalidateAll();
  }

  async function populate() {
    try {
      await maint.syncServiceLogs();
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
      message: maint.slogSuccessSyncMsg(),
      timeout: 3000,
      hoverable: true
    });
    invalidateAll();
  }
</script>

<Perm perms="system.change_service_log">
  <div class="my-4">
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
</Perm>

<div class="table-container my-4">
  <table class="table table-hover">
    <thead>
      <tr>
        <th>Report No.</th>
        <th>Title</th>
        <th>Type</th>
        <th>Date</th>
        <th>Mileage (𝗆)</th>
        <th>Progress Comment</th>
        <th>Next Milestone</th>
      </tr>
    </thead>
    <tbody>
      {#each data.data.service_logs as slog, i}
        <tr>
          <td>{slog.report_no || '-'}</td>
          <td>{slog.title}</td>
          <td>{_.startCase(slog.type)}</td>
          <td>{slog.start_time?.substring(0, 10) || '-'}</td>
          <td>{slog.mileage ?? '-'}</td>
          <td class="!whitespace-pre-line">{slog.progress_comment || '-'}</td>
          <td class="!whitespace-pre-line">{slog.next_milestone || '-'}</td>
        </tr>
      {:else}
        <tr>
          <td colspan="7">No service logs.</td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
