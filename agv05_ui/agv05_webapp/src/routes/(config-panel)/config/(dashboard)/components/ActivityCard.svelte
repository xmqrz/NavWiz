<script>
  import Pie from 'components/Pie.svelte';

  export let dashboard = {};
  let pieData = {};
  function loadPieData() {
    if (!dashboard.stats || !dashboard.stats.agv_activity) {
      return;
    }

    let total = Object.values(dashboard.stats.agv_activity).reduce((acc, v) => acc + v, 0);

    pieData = Object.fromEntries(
      Object.entries(dashboard.stats.agv_activity).map(([k, v]) => [
        k,
        Math.round((v / total) * 100.0)
      ])
    );
  }
  loadPieData();
</script>

<div class="card h-full w-full shadow-lg">
  <section class="flex items-center justify-center p-7">
    <Pie data={pieData} />
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">AGV ACTIVITIES</div>
  </footer>
</div>
