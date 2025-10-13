<script>
  import bb, { pie as bbPie } from 'billboard.js';

  // NOTE: pie cannot handle flex size that well
  export let data = {};
  export let formatDuration = false;

  // display duration
  function dd(seconds) {
    if (!seconds) {
      return '-';
    }
    let hours = Math.floor(seconds / 3600);
    let minutes = Math.floor((seconds % 3600) / 60);
    seconds %= 60;
    return `${hours}:${('0' + minutes).slice(-2)}:${('0' + seconds).slice(-2)}`;
  }

  function pie(dom, initData) {
    var chart;
    function genChart(d) {
      chart = bb.generate({
        data: {
          columns: Object.entries(d),
          type: bbPie()
        },
        bindto: dom,
        tooltip: {
          format: {
            // By default is return ratio
            value: function (value, _ratio, _id, _index) {
              if (formatDuration) {
                return dd(value);
              }
              return value.toFixed(2);
            }
          }
        }
      });
    }

    genChart(initData);

    return {
      update(newData) {
        chart.destroy();
        genChart(newData);
      },
      destroy() {
        chart.destroy();
      }
    };
  }
</script>

<div class="h-full w-full" use:pie={data}></div>
