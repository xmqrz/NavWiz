<script>
  import { onMount, getContext, setContext } from 'svelte';
  import { writable, readonly } from 'svelte/store';

  const mainController = getContext('mainController');
  const moduleManager = mainController.moduleService.getManager();

  const activeModuleVar = writable('-');
  const activeModule = readonly(activeModuleVar);
  setContext('activeModule', activeModule);

  function moduleManagerCallback(data) {
    if (data.command === 'active_module') {
      activeModuleVar.set(data.module_id);
    }
  }

  onMount(() => {
    moduleManager.subscribe(moduleManagerCallback);
    // allow time for subscribe to happen first.
    setTimeout(function () {
      moduleManager.publish({
        command: 'active_module'
      });
    }, 100);

    return () => {
      moduleManager.unsubscribe(moduleManagerCallback);
    };
  });
</script>

<slot />
