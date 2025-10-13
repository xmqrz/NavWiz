<script>
  import { onDestroy } from 'svelte';
  import { AppShell } from '@skeletonlabs/skeleton';
  import { getContext } from 'svelte';
  import { page } from '$app/stores';

  import { robotRunning, robotName } from 'stores/states.js';
  import Perm from 'components/Perm.svelte';
  import SideRail from './components/SideRail.svelte';
  import DisconnectedIndicator from './components/DisconnectedIndicator.svelte';
  import TopBar from './components/TopBar.svelte';
  import { whiteLabel } from 'stores/white-label.js';
  import Keyboard from 'components/keyboard/Keyboard.svelte';

  const dashboardId = '/(user-panel)/(controller)/dashboard';
  const mainController = getContext('mainController');
  mainController.spin();

  onDestroy(() => {
    mainController.stop();
  });
</script>

<svelte:head>
  <title>NavWiz UserPanel{$robotName ? ` (${$robotName})` : ''}</title>
</svelte:head>

<DisconnectedIndicator />
<AppShell slotSidebarLeft="w-auto shadow-xl scrollbar-hide">
  <svelte:fragment slot="header">
    <TopBar />
  </svelte:fragment>
  <svelte:fragment slot="sidebarLeft">
    <Perm perms={['app.show_panel_side_menu']}>
      {#if $robotRunning === 1}
        <SideRail />
      {/if}
    </Perm>
  </svelte:fragment>

  <div class="h-full p-4">
    <slot />
  </div>

  <svelte:fragment slot="pageFooter">
    {#if $page.route.id !== dashboardId}
      <span class="float-start mb-1 pl-3 text-xs text-gray-400 md:text-sm">
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        {@html $whiteLabel.copyright_label || ''}
      </span>
      <span class="float-right mb-1 pr-3 text-xs text-gray-400 md:text-sm">
        NavWiz {$whiteLabel.navwiz_version || ''}
      </span>
    {/if}
  </svelte:fragment>
</AppShell>
<Keyboard />
