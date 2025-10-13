<script>
  import bb, { bar as bbBar } from 'billboard.js';
  export let data = [];
  export let formatDuration = false;
  export let groups = [];
  export let yAxisLabel = '';

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

  function bar(dom, initData) {
    var chart;
    function genBar(d) {
      chart = bb.generate({
        data: {
          x: 'date',
          columns: d,
          type: bbBar(),
          groups: groups
        },
        axis: {
          x: {
            type: 'timeseries',
            tick: {
              format: '%d %B'
            }
          },
          y: {
            label: {
              text: yAxisLabel,
              position: 'outer-middle'
            },
            tick: {
              format: function (x) {
                if (formatDuration) {
                  return dd(x);
                } else {
                  return x;
                }
              }
            }
          }
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

    genBar(initData);

    return {
      update(newData) {
        chart.destroy();
        genBar(newData);
      },
      destroy() {
        chart.destroy();
      }
    };
  }
</script>

<div class="h-full w-full" use:bar={data}></div>
