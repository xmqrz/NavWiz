<script>
  import { popup } from '@skeletonlabs/skeleton';

  export let src = '';
  export let skillset = '';
  export let placement = 'bottom';
  export let offset = -25;

  const imgPopup = {
    event: 'hover',
    target: 'imgPopup',
    placement: placement,
    middleware: {
      offset
    }
  };

  $: skillsetData = skillset ? JSON.parse(skillset) : { summary: 'not available' };
</script>

<div class="h-32 w-32 rounded-3xl [&>*]:pointer-events-none" use:popup={imgPopup}>
  <img class="rounded-3xl" alt="skillset" {src} />
</div>
<div
  class="card bg-surface-50-900-token z-50 m-10 max-w-4xl whitespace-pre-line p-4 shadow-xl"
  data-popup="imgPopup">
  {skillsetData.summary}
</div>
