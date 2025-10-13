<script>
  import { setContext } from 'svelte';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';
  import { page } from '$app/stores';
  import { postAPI } from '$lib/utils';
  import Perm from 'components/Perm.svelte';

  const modalStore = getModalStore();
  const titles = [
    'Network Connections',
    'Edit Network Configuration',
    'Edit Hostname',
    'Edit HTTPS Settings',
    'Edit EAP Settings'
  ];
  const urls = ['/config/network', 'configuration', 'hostname', 'https', 'eap'];
  $: index = getIndex($page.url.pathname);

  function getIndex(url) {
    if (url.endsWith('/')) {
      url = url.slice(0, -1);
    }
    const suffix = url.split('/').at(-1);
    for (let i = 1; i < urls.length; i++) {
      if (suffix === urls[i]) return i;
    }
    if (url !== urls[0]) {
      goto(urls[0]);
    }
    return 0;
  }

  async function onSubmit(data, back = true) {
    let url = $page.url.pathname;
    if (url.endsWith('/')) {
      url = url.slice(0, -1);
    }
    let res = await postAPI(url, data);
    if (back && !('error' in res && Object.keys(res.error).length)) {
      goto(urls[0]);
    }
    return res;
  }

  function onIdentify(name, update) {
    if (!update(false, name)) return;

    const onIdentifyAsync = async (n) => {
      const res = await postAPI(urls[0] + '/identify', { name: n });
      if ('name' in res) {
        update(true, n);
      } else {
        update(true);
        modalStore.trigger({
          type: 'alert',
          title: 'Identify ' + n,
          body: n + ' is busy.'
        });
      }
    };
    onIdentifyAsync(name);
  }

  setContext('networkSubmit', onSubmit);
  setContext('networkIdentify', onIdentify);
</script>

<ConfigLayout title={titles[index]} back={index ? ['Back', urls[0]] : []} validation={false}>
  {#if index === 0}
    <Perm perms="system.change_network">
      <div>
        {#each titles as t, i}
          {#if i}
            <a href={urls[0] + '/' + urls[i]} class="variant-filled-secondary btn m-1">
              {t}
              <i class="fa-solid fa-up-right-from-square ml-2"></i>
            </a>
          {/if}
        {/each}
      </div>

      <div class="py-10">
        <hr />
      </div>
    </Perm>
  {/if}

  <div>
    <slot />
  </div>
</ConfigLayout>
