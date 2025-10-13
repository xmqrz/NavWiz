<script>
  import { getModalStore, ProgressRadial } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';
  import { getEnv } from 'stores/auth';

  export let data = {};

  const user = getContext('user');
  const modalStore = getModalStore();
  const mainController = getContext('mainController');
  const moduleManager = mainController.moduleService.getManager();

  let hwApp;
  let hwAppChannel;
  let init;
  let destroy;

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: `Confirm stopping module<br/>"${hwApp ? hwApp.name : data.app_id}" ?`,
      response: (r) => {
        if (r) {
          moduleManager.stopModule(data.app_id);
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  onMount(() => {
    moduleManager.getHwApp(data.app_id).then(function (d) {
      hwAppChannel = mainController.moduleService.getSub(data.app_id);
      hwApp = d;
    });
    return () => {
      if (destroy) {
        destroy();
      }
    };
  });

  function navwizAPI(dom) {
    const appAPI = {
      publish: hwAppChannel.publish.bind(hwAppChannel),
      subscribe: hwAppChannel.subscribe.bind(hwAppChannel),
      unsubscribe: hwAppChannel.unsubscribe.bind(hwAppChannel),
      isAgvPanel: () => $user.is_agv_panel(),
      isTrackless: () => getEnv('TRACKLESS')
    };

    const fn = init(dom, appAPI);
    if (fn instanceof Function) {
      destroy = fn;
    }
  }

  function load(hwApp) {
    if (hwApp === undefined || init) {
      return;
    }
    import(/* @vite-ignore */ hwApp.entry)
      .then((module) => {
        init = module.init;
      })
      .catch((error) => {
        console.error('Error loading HWApp module:', error);
      });
  }
  $: load(hwApp);
</script>

<AppLayout title={hwApp ? hwApp.name : ''} moduleID={data.app_id} on:close={shutdown}>
  {#if !init}
    <ProgressRadial width="w-24 p-5" />
  {:else}
    <div use:navwizAPI></div>
  {/if}
</AppLayout>
