<script>
  import { onMount } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';

  import { robotRunning } from 'stores/states.js';

  export let activeModule;

  const modalStore = getModalStore();

  function onRobotRunning(mode) {
    if (mode !== 1) {
      modalStore.close();
    }
  }

  function onActiveModule(am) {
    if (am !== 'task-runner') {
      modalStore.close();
    }
  }

  onMount(() => {
    const unsubscribe = robotRunning.subscribe(onRobotRunning);
    const unsubscribeAM = activeModule.subscribe(onActiveModule);
    return () => {
      unsubscribe();
      unsubscribeAM();
    };
  });
</script>

<slot />
