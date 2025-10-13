<script>
  import { TabGroup, Tab } from '@skeletonlabs/skeleton';
  import { format } from 'date-fns';

  import { csvReport } from './csv-report';
  import CombinedTaskCompletedPieChart from './CombinedTaskCompletedPieChart.svelte';
  import AverageCompletedTaskDurationBarChart from './AverageCompletedTaskDurationBarChart.svelte';
  import CountTaskCompletedPieChart from './CountTaskCompletedPieChart.svelte';
  import CountTaskCompletedBarChart from './CountTaskCompletedBarChart.svelte';
  import DurationTaskCompletedPieChart from './DurationTaskCompletedPieChart.svelte';
  import DurationTaskCompletedBarChart from './DurationTaskCompletedBarChart.svelte';

  export let data;

  const statuses = data.statuses;
  let tab = 'combined';

  // display month
  function dm(month) {
    return format(month, 'MMMM yyyy');
  }

  // display duration
  function dd(seconds) {
    if (!seconds) {
      return '-';
    }
    let hours = Math.floor(seconds / 3600);
    let minutes = Math.floor((seconds % 3600) / 60);
    seconds %= 60;
    return `${hours}:${('0' + minutes).slice(-2)}:${('0' + seconds).slice(-2)}`;
  }

  const aPlaceholder = 'blob:report.csv';

  function download(evt) {
    let a = evt.target;

    let blob = new Blob([csvReport(data, tab)], { type: 'application/csv' });
    let url = window.URL.createObjectURL(blob);

    a.href = url;
    a.download = format(data.current, 'yyyy_MM') + '_report.csv';

    setTimeout(() => {
      a.href = aPlaceholder;
      window.URL.revokeObjectURL(url);
    }, 150);
  }
</script>

<div class="space-y-4 py-4">
  <p>Last updated on: {data.lastUpdate.toLocaleString()}</p>
  <div class="table-container">
    <table class="table">
      <thead>
        <tr>
          <th class="!p-2 text-center">
            {#if data.previous}
              <a class="float-left" href="/config/task-completed/report/{data.previous}">
                <i class="fa fa-lg fa-arrow-circle-left"></i>
              </a>
            {/if}
            {dm(data.current)}
            {#if data.next}
              <a class="float-right" href="/config/task-completed/report/{data.next}">
                <i class="fa fa-lg fa-arrow-circle-right"></i>
              </a>
            {/if}
          </th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td class="!p-0">
            <TabGroup flex="flex-auto">
              <Tab bind:group={tab} name="combined" value="combined">Combined</Tab>
              <Tab bind:group={tab} name="count" value="count">Count</Tab>
              <Tab bind:group={tab} name="duration" value="duration">Duration</Tab>
            </TabGroup>
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</div>

{#if tab === 'combined'}
  <CombinedTaskCompletedPieChart taskStatistics={data} />
  <AverageCompletedTaskDurationBarChart taskStatistics={data} />
{:else if tab === 'count'}
  <CountTaskCompletedBarChart taskStatistics={data} />
  <CountTaskCompletedPieChart taskStatistics={data} />
{:else if tab === 'duration'}
  <DurationTaskCompletedBarChart taskStatistics={data} />
  <DurationTaskCompletedPieChart taskStatistics={data} />
{/if}

<div class="table-container pt-4">
  <table class="table">
    <thead>
      <tr>
        <th width="150px">Status</th>
        {#each data.days as day}
          <th>{day + 1}</th>
        {/each}
        <th>Total</th>
      </tr>
    </thead>
    <tbody>
      {#each statuses as st}
        <tr>
          <td rowspan={tab === 'combined' ? 2 : 1}>{st}</td>
          {#if tab !== 'duration'}
            {#each data.byStatus[st] as datum}
              <td align="right">{datum?.count || '-'}</td>
            {/each}
          {:else}
            {#each data.byStatus[st] as datum}
              <td align="right">{dd(datum?.duration)}</td>
            {/each}
          {/if}
        </tr>
        {#if tab === 'combined'}
          <tr>
            {#each data.byStatus[st] as datum}
              <td align="right">{dd(datum?.duration)}</td>
            {/each}
          </tr>
        {/if}
      {/each}
      <tr><td colspan="50">&nbsp;</td></tr>
    </tbody>
  </table>
</div>

{#each statuses as st}
  <div class="table-container pt-4">
    <table class="table">
      <thead>
        <tr><th colspan="50"><i>Breakdown: {st} Tasks</i></th></tr>
      </thead>
      <thead>
        <tr>
          <th>Task Template</th>
          {#each data.days as day}
            <th>{day + 1}</th>
          {/each}
          <th>Total</th>
        </tr>
      </thead>
      <tbody>
        {#each data.byTt[st] as tt}
          <tr>
            <td rowspan={tab === 'combined' ? 2 : 1}>{tt.taskTemplate}</td>
            {#if tab !== 'duration'}
              {#each tt.data as datum}
                <td align="right">{datum?.count || '-'}</td>
              {/each}
            {:else}
              {#each tt.data as datum}
                <td align="right">{dd(datum?.duration)}</td>
              {/each}
            {/if}
          </tr>
          {#if tab === 'combined'}
            <tr>
              {#each tt.data as datum}
                <td align="right">{dd(datum?.duration)}</td>
              {/each}
            </tr>
          {/if}
        {:else}
          <tr><td colspan="50">No data.</td></tr>
        {/each}
      </tbody>
    </table>
  </div>
{/each}

<div class="pt-3">
  <a class="anchor" href={aPlaceholder} on:click={download}>
    <i class="fa fa-download"></i>
    Download report ({tab}) as csv
  </a>
</div>
