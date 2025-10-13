<script>
  import { popup } from '@skeletonlabs/skeleton';

  export let src = '';
  export let name = '';
  export let version = '';
  export let placement = 'top';

  const imgPopup = {
    event: 'hover',
    target: 'imgPopup',
    placement: placement,
    middleware: {
      offset: 20
    }
  };
</script>

<div class="h-32 w-32 [&>*]:pointer-events-none" use:popup={imgPopup}>
  <img class="rounded-3xl" alt="skillset" {src} />
</div>
<div
  class="bg-surface-50-900 card z-50 max-w-4xl whitespace-pre-line p-4 shadow-xl"
  data-popup="imgPopup">
  <span class="block text-center text-lg font-bold tracking-wide"> Restore File Info </span>
  <span class="block font-mono">Name: {name}</span>
  <span class="block font-mono">Version: {version}</span>
</div>
