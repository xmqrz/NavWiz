<script>
  import { TabAnchor, TabGroup } from '@skeletonlabs/skeleton';
  import { page } from '$app/stores';

  export let urlPrefix = '';
  export let navs = [];
</script>

<TabGroup>
  {#each navs as nav}
    <TabAnchor
      href={urlPrefix + nav.href}
      selected={$page.url.pathname === urlPrefix + nav.href ||
        $page.url.pathname === urlPrefix + nav.href + '/' ||
        (nav.expandable && $page.url.pathname.startsWith(urlPrefix + nav.href))}>
      <i class="fa {nav.icon}"></i>
      {nav.name}
    </TabAnchor>
  {/each}
</TabGroup>
