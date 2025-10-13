<script>
  import Bar from 'components/Bar.svelte';

  export let agvStatistics = {};
  let dailyTaskCountBarData = [];
  let noData = false;

  function loadBarData() {
    dailyTaskCountBarData = [];
    noData = false;
    if (!agvStatistics.barChartDailyTaskCountData) {
      noData = true;
      return;
    }
    dailyTaskCountBarData = agvStatistics.barChartDailyTaskCountData;
  }
  $: loadBarData(agvStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Bar data={dailyTaskCountBarData} yAxisLabel="Count" />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Daily task count</div>
  </footer>
</div>
