<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import editorF from './editor.js';

  export let data;

  const modalStore = getModalStore();
</script>

<form class="agv05x-map relative" on:submit|preventDefault novalidate>
  <div
    class="editor"
    editable="true"
    resizable="true"
    grid="true"
    changesets="true"
    dynamic={data.layout.dynamic || 'false'}
    on:ready
    on:dirty
    on:save
    use:editorF={[data, modalStore]}>
  </div>
  <div class="title-info">
    <strong>Map:</strong>
    {data.layout.name}
  </div>
</form>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
  @import '$lib/styles/agv05-toolbar.css';
</style>
