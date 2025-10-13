<script>
  import Bar from 'components/Bar.svelte';

  export let agvStatistics = {};
  let dailyActivityDurationBarData = [];
  let barChartGroups = [];
  let noData = false;

  function loadBarData() {
    dailyActivityDurationBarData = [];
    barChartGroups = [];
    noData = false;
    if (!agvStatistics.barChartDurationData) {
      noData = true;
      return;
    }
    dailyActivityDurationBarData = agvStatistics.barChartDurationData;
    barChartGroups = [agvStatistics.activityList];
  }
  $: loadBarData(agvStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Bar
        data={dailyActivityDurationBarData}
        groups={barChartGroups}
        yAxisLabel="Duration"
        formatDuration={true} />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Daily activity breakdown</div>
  </footer>
</div>
