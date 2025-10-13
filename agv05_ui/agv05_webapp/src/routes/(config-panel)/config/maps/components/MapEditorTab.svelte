<script>
  import maps from '$lib/shared/services/config/maps';

  import { getEnv } from 'stores/auth';

  export let mapID;
  export let curMode;

  const activeClass = 'variant-filled-secondary pointer-events-none';
</script>

<div class="variant-filled btn-group mb-5">
  {#if getEnv('TRACKLESS')}
    <a class={curMode === 'ocg' ? activeClass : ''} href={maps.ocgsUrl(mapID)}>
      <i class="fa-solid fa-image pr-1"></i>
      Raw Maps
    </a>
  {/if}
  <a class={curMode === 'layout' ? activeClass : ''} href={maps.layoutUrl(mapID)}>
    <i class="fa-solid fa-map pr-1"></i>
    Map Layout
  </a>
  <a class={curMode === 'annotation' ? activeClass : ''} href={maps.annotationUrl(mapID)}>
    <i class="fa-solid fa-commenting pr-1"></i>
    Map Annotation
  </a>
  <a class={curMode === 'edit' ? activeClass : ''} href={maps.editUrl(mapID)}>
    <i class="fa-solid fa-pencil pr-1"></i>
    Properties
  </a>
</div>
