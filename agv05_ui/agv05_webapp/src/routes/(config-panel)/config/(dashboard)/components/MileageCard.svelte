<script>
  import { toServerTZ } from 'stores/server-clock.js';
  import Graph from 'components/Graph.svelte';
  export let agv = {};
  export let dashboard = {};

  let recentMileage = '-';

  let graphData = {
    plot: [],
    legend: []
  };
  function loadGraphData() {
    if (!dashboard.stats || !dashboard.stats.mileage) {
      return;
    }
    graphData.plot.push(
      dashboard.stats.mileage.map((data) => {
        let date = new Date(data[0]);
        date = toServerTZ(date, true);
        return [date, data[1]];
      })
    );
    graphData.legend.push('mileage');

    const min = dashboard.stats.mileage[0][1];
    const max = dashboard.stats.mileage[dashboard.stats.mileage.length - 1][1];
    if (max >= min) {
      recentMileage = max - min;
    }
  }
  loadGraphData();
</script>

<div class="card flex h-full w-full flex-row shadow-lg">
  <div class="flex-[2] pl-2">
    <Graph data={graphData} graphClass="min-h-[250px]" marginLeft="60" />
  </div>
  <div
    class="flex min-w-[160px] flex-1 flex-col justify-center gap-4 rounded-lg rounded-tl-[60px] bg-gradient-to-b from-secondary-500 to-secondary-600 p-1 p-4 text-white">
    <div class="text-center text-lg font-light tracking-widest">MILEAGE</div>
    <div>
      <div class="text-center text-sm tracking-wider opacity-50">Current Mileage</div>
      <div class="text-md text-center font-bold">{agv.mileage ?? '-'} m</div>
    </div>
    <div>
      <div class="text-center text-sm tracking-wider opacity-50">Recent Mileage</div>
      <div class="text-md text-center font-bold">{recentMileage ?? '-'} m</div>
    </div>
  </div>
</div>
