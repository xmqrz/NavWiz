<script>
  import { AppBar } from '@skeletonlabs/skeleton';
</script>

<AppBar
  gridColumns="grid-cols-2"
  slotDefault="place-self-center"
  slotTrail="place-content-end"
  background="shadow-lg bg-primary-600-300-token">
  <svelte:fragment slot="trail">
    <button type="button" class="variant-filled-error btn px-2.5">
      <i class="fa-solid fa-power-off m-1"></i>
    </button>
  </svelte:fragment>
</AppBar>
