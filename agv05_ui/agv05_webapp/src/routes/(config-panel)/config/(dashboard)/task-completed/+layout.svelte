<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import NavTab from 'components/NavTab.svelte';

  let urlPrefix = '/config/task-completed';
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
    }
  ];
</script>

<ConfigLayout title="Completed Tasks" validation={false}>
  <NavTab {urlPrefix} {navs} />
  <slot />
</ConfigLayout>
