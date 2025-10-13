<script>
  import { AppRail, AppRailAnchor } from '@skeletonlabs/skeleton';

  // TODO: add i18n
  const railAnchors = [
    {
      href: '/dashboard',
      text: 'Dashboard',
      title: 'Dashboard',
      icon: 'fa-tv',
      perms: []
    },
    {
      href: '/apps',
      text: 'Apps',
      title: 'Apps',
      icon: 'fa-cube',
      perms: ['app.show_panel_all_modules']
    },
    {
      href: '/map',
      text: 'Map',
      title: 'Map',
      icon: 'fa-map',
      perms: ['app.show_panel_live_map']
    },
    {
      href: '/camera',
      text: 'Camera',
      title: 'Camera',
      icon: 'fa-camera',
      perms: ['app.show_panel_live_camera']
    },
    {
      href: '/status',
      text: 'Status',
      title: 'Status',
      icon: 'fa-chart-simple',
      perms: ['app.show_panel_health_status']
    },
    {
      href: '/custom-log',
      text: 'Custom Log',
      title: 'Custom Log',
      icon: 'fa-file',
      perms: ['app.show_panel_health_status']
    },
    {
      href: '/help',
      text: 'Help',
      title: 'Help',
      icon: 'fa-life-ring',
      perms: []
    }
  ];
</script>

<AppRail height="min-h-full">
  {#each railAnchors as a, i}
    <div class="anchor-target">
      <AppRailAnchor
        hover="bg-primary-hover-token"
        active="!bg-primary-400"
        title={a.title}
        selected={i == 0}>
        <svelte:fragment slot="lead">
          <i class="fa-solid custom-text {a.icon}"></i>
        </svelte:fragment>
        <span>{a.text}</span>
      </AppRailAnchor>
    </div>
  {/each}
</AppRail>

<style lang="postcss">
  .anchor-target :global(.app-rail-wrapper) {
    @apply aspect-[4/3];
  }
  @media (min-height: 560px) {
    .anchor-target :global(.app-rail-wrapper) {
      @apply aspect-[6/7];
    }
    .custom-text {
      @apply text-2xl;
    }
  }
</style>
