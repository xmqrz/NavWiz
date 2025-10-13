<script>
  import { writable } from 'svelte/store';
  import { DateInput } from 'date-picker-svelte';
  import { format } from 'date-fns';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { isBefore } from 'date-fns';

  import log from '$lib/shared/services/config/log.js';

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';

  const modalStore = getModalStore();

  let form;
  let start = new Date();
  let end = start;
  let error = {};
  let timeout, spinner;

  function onDownload() {
    if (isBefore(end, start)) {
      error = {
        start: 'Start date must be before end date.'
      };
      return;
    }
    error = {};
    log
      .requestSystemLogs({
        start_date: format(start, 'yyyy-MM-dd'),
        end_date: format(end, 'yyyy-MM-dd')
      })
      .then((r) => {
        cancelSpinner();
        spinner = {
          type: 'component',
          component: 'modalLoadingSpinner',
          meta: {
            contentStore: writable('<div class="text-center">Preparing log file<br/></div>')
          }
        };
        modalStore.trigger(spinner);
        startMonitorDownloadRequest(r.request_id);
      })
      .catch(async (err) => {
        try {
          const data = await err.cause.json();
          if (data.start_date) {
            error = {
              start: data.start_date
            };
          } else if (data.end_date) {
            error = {
              end: data.end_date
            };
          }
        } catch (e) {
          console.log('Fail to process error response.');
        }
      });
  }

  function startMonitorDownloadRequest(request_id) {
    timeout = setTimeout(() => {
      monitorDownloadRequest(request_id);
    }, 1000);
  }

  function cancelSpinner() {
    if (spinner) {
      modalStore.close(spinner);
      spinner = undefined;
    }
    if (timeout) {
      clearTimeout(timeout);
      timeout = undefined;
    }
  }

  function monitorDownloadRequest(request_id) {
    timeout = undefined;
    log
      .checkSystemLogs({
        request_id: request_id
      })
      .then((r) => {
        if (r.status === 'READY') {
          cancelSpinner();
          log.downloadSystemLogs({
            request_id: request_id
          });
        } else if (r.status === 'FAILURE') {
          cancelSpinner();
          alert('Fail to download: server error.');
        } else {
          if (spinner && r.size) {
            spinner.meta.contentStore.set(
              `<div class="text-center">Preparing log file<br/>(${r.size})</div>`
            );
          }
          startMonitorDownloadRequest(request_id);
        }
      })
      .catch(() => {
        startMonitorDownloadRequest(request_id);
      });
  }
</script>

<div class="flex space-x-5 py-5">
  <h2 class="text-xl tracking-widest">Download Log</h2>
</div>
<hr />
<br />
<form bind:this={form} on:submit|preventDefault class="container space-y-2">
  <div class="grid grid-cols-4 items-center p-3">
    <span class={formLabelClass} class:text-red-500={error.start}>Start Date</span>
    <div class="col-span-3">
      <div class="max-w-max">
        <DateInput
          bind:value={start}
          format="yyyy-MM-dd"
          class={error.start ? 'parent-input-error' : ''} />
      </div>
    </div>
    {#if error.start}
      <span class="col-span-3 col-start-2 text-red-500">{error.start}</span>
    {/if}
  </div>
  <div class="grid grid-cols-4 items-center p-3">
    <span class={formLabelClass}>End Date</span>
    <div class="col-span-3">
      <div class="max-w-max">
        <DateInput bind:value={end} format="yyyy-MM-dd" />
      </div>
    </div>
  </div>
  <div class="grid grid-cols-4 items-center p-3">
    <div class="col-span-3 col-start-2">
      <button type="submit" class="variant-filled btn" on:click={onDownload}>
        Download
      </button>
    </div>
  </div>
</form>
