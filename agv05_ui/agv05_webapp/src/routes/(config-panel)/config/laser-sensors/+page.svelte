<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { invalidateAll, beforeNavigate } from '$app/navigation';

  import laserSensors from '$lib/shared/services/config/laser-sensors';
  import editorF from './editor.js';

  export let data;

  const toastStore = getToastStore();

  let viz;
  let dirty = false;

  function editorReady(e) {
    viz = e.target.viz;
  }

  function editorDirty() {
    dirty = true;
  }

  function editorSave() {
    save();
  }

  function save() {
    if (!viz) {
      return;
    }
    var payload = viz.editor.stage();

    laserSensors
      .update(payload)
      .then((_d) => {
        toastStore.trigger({
          message: laserSensors.successUpdateMsg(),
          timeout: 3000,
          hoverable: true
        });
        muteUnload();
        // Prevent using preloaded data
        invalidateAll();
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function muteUnload() {
    dirty = false;
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });
</script>

<ConfigLayout title="Laser Sensors" validation={false}>
  <div
    class="editor"
    on:ready={editorReady}
    on:dirty={editorDirty}
    on:save={editorSave}
    use:editorF={data}>
  </div>
</ConfigLayout>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
  @import '$lib/styles/agv05-laser-sensors.css';
</style>
