<script>
  import Bar from 'components/Bar.svelte';

  export let agvStatistics = {};
  let dailyActivityPercentageBarData = [];
  let barChartGroups = [];
  let noData = false;

  function loadBarData() {
    dailyActivityPercentageBarData = [];
    barChartGroups = [];
    noData = false;
    if (!agvStatistics.barChartPercentageData) {
      noData = true;
      return;
    }
    dailyActivityPercentageBarData = agvStatistics.barChartPercentageData;
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
        data={dailyActivityPercentageBarData}
        groups={barChartGroups}
        yAxisLabel="Percentage" />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Daily activity breakdown</div>
  </footer>
</div>
