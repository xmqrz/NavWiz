<script>
  import { popup } from '@skeletonlabs/skeleton';
  import * as _ from 'lodash-es';

  export let sensor = '';
  export let markerType = '';
  export let detecting = false;
  export let publish;

  const markerTypePopup = {
    event: 'click',
    target: 'marker-type',
    placement: 'bottom',
    closeQuery: '.close'
  };

  const profiles = ['pallet', 'pallet2', 'pallet3', 'pallet4', 'pallet5'];
  let profile = profiles[0];
  let suffix = 'r4';

  function stop() {
    publish('');
  }

  function trigger() {
    publish(`${profile}__${sensor}${suffix && '__' + suffix}`);
  }
</script>

<span
  class="chip variant-{detecting ? 'filled-warning' : 'ghost'}"
  title={detecting ? 'Active' : 'Inactive'}
  use:popup={markerTypePopup}>
  Marker Type: {markerType || '-'}
</span>
<div class="card z-30 space-y-2 p-2 text-sm shadow-xl" data-popup="marker-type">
  <div class="flex items-center gap-2">
    <span class="w-12">Profile:</span>
    <select class="chip flex-1 text-left" bind:value={profile}>
      {#each profiles as p}
        <option value={p}>{_.startCase(p)}</option>
      {/each}
    </select>
  </div>
  <div class="flex items-center gap-2">
    <span class="w-12">Suffix:</span>
    <input class="chip flex-1 text-left" placeholder="(optional)" bind:value={suffix} />
  </div>
  <div class="flex gap-2">
    <button class="close variant-filled btn chip flex-1" on:click={stop}>Stop</button>
    <button class="close variant-filled btn chip flex-1" on:click={trigger}>Trigger</button>
  </div>
</div>
