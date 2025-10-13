<script>
  import { page } from '$app/stores';

  import LogDirTable from '../../components/LogDirTable.svelte';

  export let data;
  $: baseUrl = $page.url.href;
</script>

<LogDirTable {data} {baseUrl} pageTitle="Peripheral Logs" defaultRawUrl="/logs/peripherals/">
</LogDirTable>
