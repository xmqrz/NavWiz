<script>
  import Bar from 'components/Bar.svelte';

  export let taskStatistics = {};
  let dailyTaskDurationBarData = [];
  let barChartGroups = [];
  let noData = false;

  function loadBarData() {
    dailyTaskDurationBarData = [];
    barChartGroups = [];
    noData = false;
    if (!taskStatistics.barChartDurationData) {
      noData = true;
      return;
    }
    dailyTaskDurationBarData = taskStatistics.barChartDurationData;
    barChartGroups = [taskStatistics.statusList];
  }
  $: loadBarData(taskStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Bar
        data={dailyTaskDurationBarData}
        groups={barChartGroups}
        formatDuration={true}
        yAxisLabel="Duration" />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Daily task duration</div>
  </footer>
</div>
