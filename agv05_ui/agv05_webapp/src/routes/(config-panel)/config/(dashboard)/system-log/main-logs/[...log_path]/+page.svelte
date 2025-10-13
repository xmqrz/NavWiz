<script>
  import { page } from '$app/stores';

  import LogDirTable from '../../components/LogDirTable.svelte';

  import { title } from 'stores/page-title';

  export let data;
  $: baseUrl = $page.url.href;

  title.set('Main Logs');
</script>

<LogDirTable {data} {baseUrl} pageTitle="Main Logs" defaultRawUrl="/logs/main/"></LogDirTable>
