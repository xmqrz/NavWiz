<script>
  import { AppRail, AppRailAnchor } from '@skeletonlabs/skeleton';
  import { page } from '$app/stores';
  import { getEnv } from 'stores/auth';

  import Perm from 'components/Perm.svelte';

  // TODO: add i18n
  var railAnchors = [
    {
      href: '/config',
      text: 'Dashboard',
      title: 'Dashboard',
      icon: 'fa-tv',
      perms: ['system.view_system_panel']
    },
    {
      href: '/config/system-monitor',
      text: 'System<br/>Monitor',
      title: 'System Monitor',
      icon: 'fa-weight-scale',
      perms: ['system.view_log_files']
    },
    {
      href: '/config/task-completed',
      text: 'Task<br/>Completed',
      title: 'Task Completed',
      icon: 'fa-list-check',
      perms: ['system.view_completed_tasks']
    },
    {
      href: '/config/agv-activities',
      text: 'Agv<br/>Activities',
      title: 'Agv Activities',
      icon: 'fa-train',
      perms: ['system.view_agv_activities']
    },
    {
      href: '/config/system-log',
      text: 'System<br/>Log',
      title: 'System Log',
      icon: 'fa-laptop-file',
      perms: ['system.view_log_files']
    }
  ];

  if (getEnv('TRACKLESS')) {
    railAnchors.push({
      href: '/config/mapx-quality',
      text: 'Map<br/>Quality<br/>Score',
      title: 'Map Quality Score',
      icon: 'fa-map-location-dot',
      perms: ['system.view_map_quality']
    });
  }

  railAnchors.push({
    href: '/config/help',
    text: 'Help',
    title: 'Help',
    icon: 'fa-circle-info',
    perms: ['system.view_help_content']
  });
</script>

<AppRail
  hover="bg-secondary-hover-token"
  active="!bg-secondary-400-500-token"
  height="min-h-full"
  border="border-r border-surface-300">
  {#each railAnchors as a, i}
    <Perm perms={a.perms}>
      <AppRailAnchor
        href={a.href}
        title={a.title}
        aspectRatio="aspect-[6/7]"
        selected={i === 0
          ? $page.url.pathname === a.href || $page.url.pathname === a.href + '/'
          : $page.url.pathname.startsWith(a.href)}>
        <svelte:fragment slot="lead">
          <i class="fa-solid text-2xl {a.icon}"></i>
        </svelte:fragment>
        <!-- eslint-disable-next-line svelte/no-at-html-tags -->
        <span>{@html a.text}</span>
      </AppRailAnchor>
    </Perm>
  {/each}
</AppRail>
