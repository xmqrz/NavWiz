<script>
  import { AppShell, ProgressRadial } from '@skeletonlabs/skeleton';
  import { setContext, onMount } from 'svelte';
  import { readable } from 'svelte/store';

  import TopBar from './components/TopBar.svelte';
  import ModalLayout from 'user-panel/(controller)/apps/(apps)/+layout.svelte';
  import AppLayout from 'components/AppLayout.svelte';
  import Keyboard from 'components/keyboard/Keyboard.svelte';
  import { initKeyboard } from 'components/keyboard/controller.js';
  import { moduleChannel } from 'stores/sock';

  let appEntry;
  let appContext = {
    name: 'HWApp Template'
  };
  let init;
  let destroy;

  // create mockup data
  setContext('activeModule', readable('hw-app'));

  function shutdown() {
    // noop
  }
  onMount(() => {
    initKeyboard();
    fetch('/api/entry')
      .then((response) => response.text())
      .then((data) => {
        appEntry = data;
      })
      .catch((error) => {
        console.error('Get entry error:', error);
      });
    fetch('/api/context')
      .then((response) => response.json())
      .then((data) => {
        appContext = data;
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

  function navwizAPI(dom) {
    const appAPI = {
      publish: moduleChannel.publish.bind(moduleChannel),
      subscribe: moduleChannel.subscribe.bind(moduleChannel),
      unsubscribe: moduleChannel.unsubscribe.bind(moduleChannel),
      isAgvPanel: () => {
        return appContext.is_agv_panel || false;
      },
      isTrackless: () => {
        return appContext.is_trackless !== undefined ? appContext.is_trackless : true;
      }
    };

    const fn = init(dom, appAPI);
    if (fn instanceof Function) {
      destroy = fn;
    }
  }

  function load(appEntry) {
    if (appEntry === undefined || init) {
      return;
    }
    import(/* @vite-ignore */ appEntry)
      .then((module) => {
        init = module.init;
      })
      .catch((error) => {
        console.error('Error loading HWApp module:', error);
      });
  }
  $: load(appEntry);
</script>

<AppShell slotSidebarLeft="w-auto shadow-xl scrollbar-hide">
  <svelte:fragment slot="header">
    <TopBar />
  </svelte:fragment>

  <div class="h-full p-4">
    <ModalLayout>
      <AppLayout
        title={appContext ? appContext.name : 'HWApp Template'}
        moduleID="hw-app"
        on:close={shutdown}>
        {#if !init}
          <ProgressRadial width="w-24 p-5" />
        {:else}
          <div use:navwizAPI></div>
        {/if}
      </AppLayout>
    </ModalLayout>
  </div>
</AppShell>
<Keyboard />
