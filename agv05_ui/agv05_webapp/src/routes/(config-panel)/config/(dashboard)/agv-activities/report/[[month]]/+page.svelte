<script>
  import { TabGroup, Tab } from '@skeletonlabs/skeleton';
  import { format } from 'date-fns';

  import { csvReport } from './csv-report';
  import ActivityByPercentageBarChart from './ActivityByPercentageBarChart.svelte';
  import ActivityByPercentagePieChart from './ActivityByPercentagePieChart.svelte';
  import ActivityByDurationBarChart from './ActivityByDurationBarChart.svelte';
  import ActivityByDurationPieChart from './ActivityByDurationPieChart.svelte';
  import DailyTravelDistanceBarChart from './DailyTravelDistanceBarChart.svelte';
  import DailyTaskCountBarChart from './DailyTaskCountBarChart.svelte';

  export let data;

  let tab = 'percentage';

  // display month
  function dm(month) {
    return format(month, 'MMMM yyyy');
  }

  // display percentage
  function dp(percentage) {
    if (!percentage) {
      return '-';
    }
    return percentage.toFixed(2) + '%';
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

  // display mileage
  function de(mileage) {
    if (mileage == null) {
      return '-';
    }
    return mileage.toFixed(1);
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
              <a class="float-left" href="/config/agv-activities/report/{data.previous}">
                <i class="fa fa-lg fa-arrow-circle-left"></i>
              </a>
            {/if}
            {dm(data.current)}
            {#if data.next}
              <a class="float-right" href="/config/agv-activities/report/{data.next}">
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
              <Tab bind:group={tab} name="percentage" value="percentage">Percentage</Tab>
              <Tab bind:group={tab} name="duration" value="duration">Duration</Tab>
              <Tab
                bind:group={tab}
                name="mileage & task counter"
                value="mileage & task counter">Mileage &amp; Task Counter</Tab>
            </TabGroup>
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</div>

{#if tab === 'percentage'}
  <ActivityByPercentageBarChart agvStatistics={data} />
  <ActivityByPercentagePieChart agvStatistics={data} />
{:else if tab === 'duration'}
  <ActivityByDurationBarChart agvStatistics={data} />
  <ActivityByDurationPieChart agvStatistics={data} />
{:else if tab === 'mileage & task counter'}
  <DailyTravelDistanceBarChart agvStatistics={data} />
  <DailyTaskCountBarChart agvStatistics={data} />
{/if}

<div class="table-container pt-4">
  {#if ['percentage', 'duration'].indexOf(tab) >= 0}
    <table class="table">
      <thead>
        <tr>
          <th width="150px">Activity</th>
          {#each data.days as day}
            <th>{day + 1}</th>
          {/each}
          <th>Total</th>
        </tr>
      </thead>
      <tbody>
        {#each data.byActivity as ac}
          <tr>
            <td>{ac.activity}</td>
            {#if tab === 'percentage'}
              {#each ac.data as datum}
                <td align="right">{dp(datum?.percentage)}</td>
              {/each}
            {:else}
              {#each ac.data as datum}
                <td align="right">{dd(datum?.duration)}</td>
              {/each}
            {/if}
          </tr>
        {:else}
          <tr><td colspan="50">No data.</td></tr>
        {/each}
      </tbody>
    </table>
  {:else}
    <table class="table">
      <thead>
        <tr>
          <th width="180px">Mileage</th>
          {#each data.days as day}
            <th>{day + 1}</th>
          {/each}
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>Absolute Mileage (m)</td>
          {#each data.extra.mileage as m}
            <td>{de(m)}</td>
          {/each}
        </tr>
        <tr>
          <td>Daily Travel Distance (m)</td>
          {#each data.extra.mileageDelta as m}
            <td>{de(m)}</td>
          {/each}
        </tr>
        <tr><td colspan="50">&nbsp;</td></tr>
      </tbody>
      <thead>
        <tr>
          <th width="180px">Task Counter</th>
          {#each data.days as day}
            <th>{day + 1}</th>
          {/each}
        </tr>
      </thead>
      <tbody>
        <tr>
          <td>Absolute Task Counter</td>
          {#each data.extra.taskCounter as t}
            <td>{t ?? '-'}</td>
          {/each}
        </tr>
        <tr>
          <td>Daily Task Count</td>
          {#each data.extra.taskCounterDelta as t}
            <td>{t ?? '-'}</td>
          {/each}
        </tr>
      </tbody>
    </table>
  {/if}
</div>

<div class="pt-3">
  <a class="anchor" href={aPlaceholder} on:click={download}>
    <i class="fa fa-download"></i>
    Download report ({tab}) as csv
  </a>
</div>
