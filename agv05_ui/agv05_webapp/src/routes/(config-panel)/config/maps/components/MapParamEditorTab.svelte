<script>
  import maps from '$lib/shared/services/config/maps';

  export let curMode;

  const activeClass = 'variant-filled-secondary pointer-events-none';
</script>

<div class="variant-filled btn-group mb-5">
  <a class={curMode === 'active' ? activeClass : ''} href={maps.activeUrl()}>
    <span class="badge"><i class="fa-solid fa-pencil"></i></span>
    Active Map
  </a>
  <a class={curMode === 'teleport' ? activeClass : ''} href={maps.teleportUrl()}>
    <span class="badge"><i class="fa-solid fa-code-compare fa-rotate-90"></i></span>
    Map Teleportation
  </a>
  <a class={curMode === 'tTrigger' ? activeClass : ''} href={maps.tTriggerUrl()}>
    <span class="badge"><i class="fa-solid fa-arrow-right-arrow-left"></i></span>
    Map Transition Trigger
  </a>
  <a class={curMode === 'param' ? activeClass : ''} href={maps.paramUrl()}>
    <span class="badge"><i class="fa-solid fa-book"></i></span>
    Map Parameters
  </a>
</div>
