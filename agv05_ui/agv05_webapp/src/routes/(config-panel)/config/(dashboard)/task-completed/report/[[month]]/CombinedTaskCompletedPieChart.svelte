<script>
  import Pie from 'components/Pie.svelte';

  export let taskStatistics = {};
  let taskCountByStatusPieData = {};
  let taskDurationByStatusPieData = {};
  let noData = false;

  function loadPieData() {
    taskCountByStatusPieData = {};
    taskDurationByStatusPieData = {};
    noData = false;

    let totalCount = Object.values(taskStatistics.total.byStatus).reduce(
      (acc, v) => acc + v.count,
      0
    );

    if (totalCount === 0) {
      noData = true;
      return;
    }

    taskCountByStatusPieData = Object.fromEntries(
      Object.entries(taskStatistics.total.byStatus).map(([k, v]) => [k, v.count])
    );

    taskDurationByStatusPieData = Object.fromEntries(
      Object.entries(taskStatistics.total.byStatus).map(([k, v]) => [k, v.duration])
    );
  }
  $: loadPieData(taskStatistics);
</script>

<div class="grid grid-cols-2 gap-4 py-4">
  <div class="card flex flex-col shadow-lg">
    <section class="flex grow items-center justify-center p-7">
      {#if noData}
        <div class="justify-center">No Data Available</div>
      {:else}
        <!-- fix when data not available then switch to available -->
        <div class="sm:h-58 h-48 xl:h-96 2xl:h-[500px]">
          <Pie data={taskCountByStatusPieData} />
        </div>
      {/if}
    </section>
    <footer class="card-footer w-full pb-7 pt-3">
      <div class="text-center text-lg font-light tracking-widest">
        Task count breakdown by status
      </div>
    </footer>
  </div>
  <div class="card flex flex-col shadow-lg">
    <section class="flex grow items-center justify-center p-7">
      {#if noData}
        <div class="justify-center">No Data Available</div>
      {:else}
        <!-- fix when data not available then switch to available -->
        <div class="sm:h-58 h-48 xl:h-96 2xl:h-[500px]">
          <Pie data={taskDurationByStatusPieData} formatDuration={true} />
        </div>
      {/if}
    </section>
    <footer class="card-footer w-full pb-7 pt-3">
      <div class="text-center text-lg font-light tracking-widest">
        Task duration breakdown by status
      </div>
    </footer>
  </div>
</div>
