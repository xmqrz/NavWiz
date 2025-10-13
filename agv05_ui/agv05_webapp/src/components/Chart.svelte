<script>
  import bb from 'billboard.js';
  export let generate = {};

  function chartAction(dom, initGen) {
    var chart;
    function genChart(d) {
      chart = bb.generate(
        Object.assign(
          {
            bindto: dom
          },
          d
        )
      );
    }

    genChart(initGen);

    return {
      update(newGen) {
        chart.destroy();
        genChart(newGen);
      },
      destroy() {
        chart.destroy();
      }
    };
  }
</script>

<div class="h-full w-full" use:chartAction={generate}></div>
