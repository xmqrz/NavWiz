<script>
  import Pie from 'components/Pie.svelte';

  export let taskStatistics = {};
  let taskDurationByTtPieData = {};
  let noData = false;

  function loadPieData() {
    taskDurationByTtPieData = {};
    noData = false;
    if (!taskStatistics.byTt) {
      return;
    }

    let totalCount = Object.values(taskStatistics.total.byTt.Completed).reduce(
      (acc, v) => acc + v.count,
      0
    );

    if (totalCount === 0) {
      noData = true;
      return;
    }

    taskDurationByTtPieData = Object.fromEntries(
      Object.values(taskStatistics.total.byTt.Completed).map((v) => [
        v.taskTemplate,
        v.duration
      ])
    );
  }
  $: loadPieData(taskStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Pie data={taskDurationByTtPieData} formatDuration={true} />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Completed task duration</div>
  </footer>
</div>
