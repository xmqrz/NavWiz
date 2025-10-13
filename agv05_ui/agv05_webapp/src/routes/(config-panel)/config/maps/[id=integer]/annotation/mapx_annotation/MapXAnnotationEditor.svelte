<script>
  import editorF from './editor.js';

  export let data;
</script>

<form class="agv05x-map" on:submit|preventDefault novalidate>
  <div
    class="editor"
    editable="true"
    resizable="true"
    grid="true"
    on:ready
    on:dirty
    on:save
    use:editorF={data}>
  </div>
  <div class="title-info">
    <strong>Map:</strong>
    {data.annotation.name}
  </div>
</form>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
  @import '$lib/styles/agv05-toolbar.css';
</style>
