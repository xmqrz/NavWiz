<script>
  import Pie from 'components/Pie.svelte';

  export let agvStatistics = {};
  let activityByPercentagePieData = {};
  let noData = false;

  function loadPieData() {
    activityByPercentagePieData = {};
    noData = false;
    if (!agvStatistics.total) {
      return;
    }

    if (!agvStatistics.total.byActivity || agvStatistics.total.byActivity.length === 0) {
      noData = true;
      return;
    }

    activityByPercentagePieData = Object.fromEntries(
      Object.entries(agvStatistics.totalActivity).map(([k, v]) => [k, v.percentage])
    );
  }
  $: loadPieData(agvStatistics);
</script>

<div class="grid gap-4 py-4">
  <div class="card flex flex-col shadow-lg">
    <section class="flex grow items-center justify-center p-7">
      {#if noData}
        <div class="justify-center">No Data Available</div>
      {:else}
        <Pie data={activityByPercentagePieData} />
      {/if}
    </section>
    <footer class="card-footer w-full pb-7 pt-3">
      <div class="text-center text-lg font-light tracking-widest">Activity breakdown</div>
    </footer>
  </div>
</div>
