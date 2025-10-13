<script>
  import { AppShell, AppBar } from '@skeletonlabs/skeleton';
  import { afterNavigate } from '$app/navigation';
  import { page } from '$app/stores';

  import UserMenu from 'components/UserMenu.svelte';
  import TimeDisplay from 'components/TimeDisplay.svelte';
  import ConfigMenu from './components/ConfigMenu.svelte';
  import SearchMenu from './components/SearchMenu.svelte';
  import SideRail from './config/(dashboard)/components/SideRail.svelte';
  import { t } from '$lib/translations';
  import { title } from 'stores/page-title';

  export let data;

  afterNavigate(() => {
    const elemPage = document.querySelector('#page');
    if (elemPage !== null) {
      elemPage.scrollTop = 0;
    }
  });
</script>

<svelte:head>
  <title>{$title}{$title ? ' | ' : ''}NavWiz ConfigPanel</title>
</svelte:head>

<AppShell slotSidebarLeft="w-auto scrollbar-hide">
  <svelte:fragment slot="header">
    <AppBar
      gridColumns="grid-cols-3"
      slotDefault="place-self-center"
      slotTrail="place-content-end"
      background="shadow-lg bg-secondary-600-300-token">
      <svelte:fragment slot="lead">
        <div class="flex flex-row text-white">
          <a href="/config" class="flex flex-col">
            <span class="text-base font-semibold md:text-xl"> NavWiz Config Panel </span>
            <span class="text-xs md:text-sm"> {data.agv.name || '-'} </span>
          </a>
        </div>
      </svelte:fragment>
      <SearchMenu />
      <svelte:fragment slot="trail">
        <ConfigMenu />
        <TimeDisplay />
        <UserMenu class="variant-filled-secondary bg-secondary-400-500-token text-black">
          <li>
            <a href="/" class="btn w-full">{$t('common.user_panel')}</a>
          </li>
        </UserMenu>
      </svelte:fragment>
    </AppBar>
  </svelte:fragment>
  <svelte:fragment slot="sidebarLeft">
    {#if $page.route.id.startsWith('/(config-panel)/config/(dashboard)')}
      <SideRail />
    {/if}
  </svelte:fragment>
  <slot />
</AppShell>
