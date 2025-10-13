<script>
  import Bar from 'components/Bar.svelte';

  export let agvStatistics = {};
  let dailyActivityPercentageBarData = [];
  let noData = false;

  function loadBarData() {
    dailyActivityPercentageBarData = [];
    noData = false;
    if (!agvStatistics.barChartDailyTravelDistanceData) {
      noData = true;
      return;
    }
    dailyActivityPercentageBarData = agvStatistics.barChartDailyTravelDistanceData;
  }
  $: loadBarData(agvStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Bar data={dailyActivityPercentageBarData} yAxisLabel="Travel Distance (m)" />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Daily travel distance</div>
  </footer>
</div>
