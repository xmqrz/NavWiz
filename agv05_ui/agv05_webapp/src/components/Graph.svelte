<script>
  import { graph } from './graph';
  export let data = {
    plot: [],
    legend: []
  };
  export let graphClass = '';
  export let marginLeft = 30;
</script>

<div class="h-full w-full {graphClass}" use:graph={{ data, graphClass, marginLeft }}></div>
