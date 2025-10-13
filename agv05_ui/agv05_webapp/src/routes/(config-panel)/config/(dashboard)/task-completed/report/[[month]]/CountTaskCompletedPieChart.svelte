<script>
  import Pie from 'components/Pie.svelte';

  export let taskStatistics = {};
  let taskCountByTtPieData = {};
  let noData = false;

  function loadPieData() {
    taskCountByTtPieData = {};
    noData = false;

    let totalCount = Object.values(taskStatistics.total.byTt.Completed).reduce(
      (acc, v) => acc + v.count,
      0
    );

    if (totalCount === 0) {
      noData = true;
      return;
    }

    taskCountByTtPieData = Object.fromEntries(
      Object.values(taskStatistics.total.byTt.Completed).map((v) => [v.taskTemplate, v.count])
    );
  }
  $: loadPieData(taskStatistics);
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Pie data={taskCountByTtPieData} />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">Completed task count</div>
  </footer>
</div>
