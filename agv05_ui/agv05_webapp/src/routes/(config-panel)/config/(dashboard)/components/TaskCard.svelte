<script>
  import { toServerTZ } from 'stores/server-clock.js';
  import Graph from 'components/Graph.svelte';
  import { add } from 'date-fns';
  export let dashboard = {};
  export let cardClass = '';

  const assignedTask = dashboard.assigned_task || 0;
  const completedTask = dashboard.completed_task || 0;
  let task_percentage = '-';
  if (!isNaN(assignedTask) && !isNaN(completedTask) && completedTask > 0) {
    task_percentage = Math.round((completedTask / assignedTask) * 100.0);
    task_percentage = Math.min(Math.max(task_percentage, 0), 100);
  }
  let graphData = {
    plot: [],
    legend: []
  };

  function loadGraphData() {
    if (!dashboard.stats) {
      return;
    }

    if (dashboard.stats.tasks) {
      graphData.plot.push(
        dashboard.stats.tasks.map((data) => {
          let date = new Date(data[0]);
          date = toServerTZ(date, true);
          return [date, data[1]];
        })
      );
      graphData.legend.push('now');
    }
    if (dashboard.stats.last_week_tasks) {
      graphData.plot.push(
        dashboard.stats.last_week_tasks.map((data) => {
          let date = add(new Date(data[0]), { days: 7 });
          date = toServerTZ(date, true);
          return [date, data[1]];
        })
      );
      graphData.legend.push('last week');
    }
  }
  loadGraphData();
</script>

<div class="card flex h-full w-full flex-row shadow-lg {cardClass}">
  <div
    class="flex flex-1 flex-col justify-center gap-4 rounded-lg rounded-br-[60px] bg-gradient-to-b from-secondary-500 to-secondary-600 p-4 text-white">
    <div class="text-center text-lg font-light tracking-widest">TODAY'S TASK</div>
    <div>
      <div class="text-center text-sm tracking-wider opacity-50">Assigned</div>
      <div class="text-md text-center font-bold">{assignedTask}</div>
    </div>
    <div>
      <div class="text-center text-sm tracking-wider opacity-50">Completed</div>
      <div class="text-md text-center font-bold">{completedTask}</div>
    </div>
    <div>
      <div class="text-center text-sm tracking-wider opacity-50">Success Rate</div>
      <div class="text-md pb-1 text-center font-bold">{task_percentage}</div>
    </div>
  </div>
  <div class="flex-[2] xl:flex-[3]">
    <Graph data={graphData} graphClass="min-h-[250px]" />
  </div>
</div>
