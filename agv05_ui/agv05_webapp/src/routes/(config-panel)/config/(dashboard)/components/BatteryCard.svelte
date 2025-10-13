<script>
  import { toServerTZ } from 'stores/server-clock.js';
  import Graph from 'components/Graph.svelte';
  export let dashboard = {};

  let graphData = {
    plot: [],
    legend: []
  };
  function loadGraphData() {
    if (!dashboard.stats || !dashboard.stats.battery) {
      return;
    }
    graphData.plot.push(
      dashboard.stats.battery.map((data) => {
        let date = new Date(data[0]);
        date = toServerTZ(date, true);
        return [date, data[1]];
      })
    );
    graphData.legend.push('battery percentage');

    // TODO: generate battery projection graph.
  }
  loadGraphData();
</script>

<div class="card h-full w-full shadow-lg">
  <header class="card-header w-full pt-7">
    <div class="text-center text-lg font-light tracking-widest">BATTERY INFORMATION</div>
  </header>
  <section>
    <Graph data={graphData} graphClass="min-h-[250px]" />
  </section>
</div>
