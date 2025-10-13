<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import NavTab from 'components/NavTab.svelte';

  let urlPrefix = '/config/agv-activities';
  let navs = [
    {
      name: 'List',
      icon: 'fa-list',
      href: '',
      expandable: false
    },
    {
      name: 'Downloads',
      icon: 'fa-download',
      href: '/downloads',
      expandable: true
    },
    {
      name: 'Report',
      icon: 'fa-book',
      href: '/report',
      expandable: true
    },
    {
      name: 'Configuration',
      icon: 'fa-cogs',
      href: '/configuration',
      expandable: false
    }
  ];
</script>

<ConfigLayout title="AGV Activities" validation={false}>
  <NavTab {urlPrefix} {navs} />
  <slot />
</ConfigLayout>
