<script>
  import { getEnv } from 'stores/auth';
  import MapController from './MapController.svelte';
  import MapXController from './MapXController.svelte';
</script>

{#if getEnv('TRACKLESS')}
  <MapXController />
{:else}
  <MapController />
{/if}
