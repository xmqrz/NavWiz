<script>
  import { AppShell, ProgressRadial, Modal, initializeStores } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';

  import TopBar from 'hwapp_src/components/TopBar.svelte';
  import Keyboard from 'components/keyboard/Keyboard.svelte';
  import SideRail from './components/SideRail.svelte';
  import { initKeyboard } from 'components/keyboard/controller.js';
  import registerCustomElement from 'user-panel/(controller)/dashboard/custom-elements';
  import { moduleService } from 'user-panel/core/module.service.js';
  import KeyboardSpacer from 'components/keyboard/KeyboardSpacer.svelte';
  import { statusAPIChannel } from 'stores/sock';

  initializeStores();

  let dashboardEntry;
  let dashboardContext = {
    name: 'Dashboard Template'
  };
  let init;
  let destroy;

  const ms = moduleService();
  const moduleManager = ms.getManager();
  const taskRunner = ms.getSub('task-runner');

  onMount(() => {
    const customElementAPI = {
      moduleManager,
      taskRunner,
      openTaskRunner,
      openTaskHistory
    };
    registerCustomElement(customElementAPI);
    initKeyboard();
    fetch('/api/entry')
      .then((response) => response.json())
      .then((data) => {
        dashboardEntry = data.entry;
      })
      .catch((error) => {
        console.error('Get entry error:', error);
      });
    fetch('/api/context')
      .then((response) => response.json())
      .then((data) => {
        dashboardContext = data;
      })
      .catch((error) => {
        console.error('Get name error:', error);
      });
    return () => {
      if (destroy) {
        destroy();
      }
    };
  });

  function dashboard(dom) {
    const dashboardAPI = {
      taskRunnerChannel: taskRunner,
      statusChannel: statusAPIChannel,
      isAgvPanel: () => {
        return dashboardContext.is_agv_panel || false;
      },
      isTrackless: () => {
        return dashboardContext.is_trackless !== undefined
          ? dashboardContext.is_trackless
          : true;
      },
      openTaskRunner,
      openTaskHistory
    };

    const fn = init(dom, dashboardAPI);
    if (fn instanceof Function) {
      destroy = fn;
    }
  }

  function openTaskRunner() {
    alert('this will open task runner');
  }

  function openTaskHistory() {
    alert('this will open task history');
  }

  function load(dashboardEntry) {
    if (dashboardEntry === undefined || init) {
      return;
    }
    import(/* @vite-ignore */ dashboardEntry)
      .then((module) => {
        init = module.init;
      })
      .catch((error) => {
        console.error('Error loading HWApp module:', error);
      });
  }
  $: load(dashboardEntry);
</script>

<Modal position="items-start" />
<AppShell slotSidebarLeft="w-auto shadow-xl scrollbar-hide">
  <svelte:fragment slot="header">
    <TopBar />
  </svelte:fragment>
  <svelte:fragment slot="sidebarLeft">
    <SideRail />
  </svelte:fragment>

  <div class="h-full p-4">
    {#if !init}
      <!-- TODO: use placeholder loading instead?... -->
      <ProgressRadial width="w-24 p-5" />
    {:else}
      <div use:dashboard></div>
    {/if}
    <KeyboardSpacer class="bg-white" />
  </div>
</AppShell>
<Keyboard />
