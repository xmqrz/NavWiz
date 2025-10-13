<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';

  import { sock } from 'stores/sock';

  const modalStore = getModalStore();
  let spinner;

  // TODO: how to override modal queue.
  onMount(() => {
    sock.on('connectionchange', onConnectionChange);

    return () => {
      sock.off('connectionchange', onConnectionChange);
      if (spinner) {
        hide();
      }
    };
  });

  function onConnectionChange(connected) {
    if (connected && spinner) {
      hide();
    } else if (!connected && !spinner) {
      show();
    }
  }

  function show() {
    spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: {
        content: 'Disconnected from agv system.<br/>Trying to reconnect...'
      }
    };
    modalStore.priorityTrigger(spinner);
  }

  function hide() {
    modalStore.close(spinner);
    spinner = undefined;
  }
</script>
