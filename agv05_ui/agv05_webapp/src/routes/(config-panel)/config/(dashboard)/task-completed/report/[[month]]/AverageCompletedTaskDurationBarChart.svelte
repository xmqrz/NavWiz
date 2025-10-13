<script>
  import { bar } from 'billboard.js';

  import Chart from 'components/Chart.svelte';

  export let taskStatistics = {};
  let noData = false;
  let generate = {};

  function loadBarData() {
    noData = false;
    generate = {};

    let totalCount = Object.values(taskStatistics.total.byTt.Completed).reduce(
      (acc, v) => acc + v.count,
      0
    );

    if (totalCount === 0) {
      noData = true;
      return;
    }

    let data = taskStatistics.total.byTt.Completed;
    let x = data.map((d) => d.taskTemplate);

    let columns = data.map((d, i) => {
      let a = Array.from({ length: data.length + 1 }, () => null);
      a[0] = d.taskTemplate;
      a[i + 1] = d.duration / d.count;
      return a;
    });

    generate = {
      data: {
        columns,
        groups: [x],
        type: bar()
      },
      axis: {
        x: {
          type: 'category',
          categories: x
        },
        y: {
          label: {
            text: 'average duration',
            position: 'outer-middle'
          },
          tick: {
            format: function (x) {
              return dd(x);
            }
          }
        }
      },
      tooltip: {
        format: {
          // By default is return ratio
          value: function (value, _ratio, _id, _index) {
            return dd(value);
          }
        }
      }
    };
  }

  $: loadBarData(taskStatistics);

  // display duration
  function dd(seconds) {
    if (!seconds) {
      return '-';
    }
    let hours = Math.floor(seconds / 3600);
    let minutes = Math.floor((seconds % 3600) / 60);
    seconds = Math.floor(seconds % 60);

    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
  }
</script>

<div class="card my-4 flex flex-col shadow-lg">
  <section class="flex grow items-center justify-center p-7">
    {#if noData}
      <div class="justify-center">No Data Available</div>
    {:else}
      <Chart {generate} />
    {/if}
  </section>
  <footer class="card-footer w-full pb-7 pt-3">
    <div class="text-center text-lg font-light tracking-widest">
      Average task duration (completed)
    </div>
  </footer>
</div>
