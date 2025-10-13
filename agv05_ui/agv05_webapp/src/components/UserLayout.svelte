<script>
  import { goto } from '$app/navigation';

  import { robotRunning } from 'stores/states.js';

  export let mode;

  function onRobotRunning(num) {
    if (num == mode) {
      return;
    }

    if (num === 0) {
      goto('/');
    } else if (num === 1) {
      goto('/dashboard');
    } else if (num === 2) {
      goto('/map-creator');
    } else if (num === 3) {
      goto('/live-app');
    }
  }

  $: onRobotRunning($robotRunning);
</script>

<!-- TODO: show loading if -1 for more then a second-->
{#if $robotRunning == mode}
  <slot />
{/if}
