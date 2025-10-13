<script>
  import { getDrawerStore } from '@skeletonlabs/skeleton';
  import { robotRunning } from 'stores/states.js';

  const drawerStore = getDrawerStore();

  function onRobotRunning(num) {
    if (num !== 1) {
      drawerStore.close();
    }
  }

  $: onRobotRunning($robotRunning);

  function closeDrawer() {
    drawerStore.close();
  }
</script>

<button on:click={closeDrawer} type="button" class="relative block h-3 w-full">
  <div
    class="bg-surface-700-200-token absolute left-[40%] top-[2px] h-[2px] w-[20%] rounded-full">
  </div>
</button>
<div class="p-3">
  {#if $robotRunning === 1}
    <slot />
  {/if}
</div>
