<script>
  import { AppBar, getModalStore } from '@skeletonlabs/skeleton';
  import { onMount } from 'svelte';
  import { page } from '$app/stores';

  import { t } from '$lib/translations';
  import { getAPI } from '$lib/utils';

  const modalStore = getModalStore();
  const logo = `${API_URL}/white-label/logo`;
  const configPage = $page.url.pathname.startsWith('/config');

  onMount(() => {
    if ($page.status >= 500 && $page.status < 600) {
      // disconnected
      return disconnectedRecovery();
    }
  });

  function disconnectedRecovery() {
    const spinner = {
      type: 'component',
      component: 'modalLoadingSpinner',
      meta: {
        content: 'Disconnected from agv system.<br/>Trying to reconnect...'
      }
    };
    modalStore.trigger(spinner);
    const timerId = setInterval(() => {
      getAPI('')
        .then((d) => {
          if (d) {
            // HACK: cannot use goto here?
            window.location.reload();
          }
        })
        .catch(() => {
          console.log('Reconnecting...');
        });
    }, 3000);
    return () => {
      modalStore.close();
      clearInterval(timerId);
    };
  }
</script>

<svelte:head>
  <title>NavWiz</title>
</svelte:head>

<AppBar
  gridColumns="grid-cols-3"
  slotDefault="place-self-center"
  slotTrail="place-content-end"
  background="shadow-lg {configPage
    ? 'bg-secondary-600-300-token'
    : 'bg-primary-600-300-token'}">
  <svelte:fragment slot="lead">
    <div class="flex flex-row">
      {#if !configPage}
        <img src={logo} class="h-8 w-16 max-w-none self-center" alt="Logo" />
      {/if}
      <a href={configPage ? '/config' : '/'} class="flex flex-col text-white">
        {#if configPage}
          <span class="text-base font-semibold md:text-xl"> NavWiz Config Panel </span>
        {:else}
          <span class="text-sm font-bold lg:text-lg">
            {$t('user_panel.title')}
          </span>
        {/if}
        <span class="text-sm md:max-w-[160px]">&nbsp;</span>
      </a>
    </div>
  </svelte:fragment>
</AppBar>
<div class="flex w-full flex-col items-center">
  <div class="pb-7 pt-24 text-5xl">Error {$page.status}</div>
  {#if $page.status === 400}
    <div>Bad request.</div>
  {:else if $page.status === 404}
    <div>Page not found.</div>
  {:else if $page.status === 500}
    <div>Server error.</div>
  {:else}
    <div>{$page.error.message}</div>
  {/if}
</div>
