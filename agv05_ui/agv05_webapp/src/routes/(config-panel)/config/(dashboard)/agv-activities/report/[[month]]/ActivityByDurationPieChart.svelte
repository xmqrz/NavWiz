<script>
  import * as _ from 'lodash-es';

  import Pie from 'components/Pie.svelte';

  export let agvStatistics = {};
  let activityByDurationPieData = {};
  let noData = false;

  function loadPieData() {
    activityByDurationPieData = {};
    noData = false;
    if (!agvStatistics.totalActivity) {
      return;
    }

    if (!agvStatistics.totalActivity || agvStatistics.totalActivity === 0) {
      noData = true;
      return;
    }

    activityByDurationPieData = Object.fromEntries(
      Object.entries(agvStatistics.totalActivity).map(([k, v]) => [k, v.duration])
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
        <Pie data={activityByDurationPieData} formatDuration={true} />
      {/if}
    </section>
    <footer class="card-footer w-full pb-7 pt-3">
      <div class="text-center text-lg font-light tracking-widest">Activity breakdown</div>
    </footer>
  </div>
</div>
