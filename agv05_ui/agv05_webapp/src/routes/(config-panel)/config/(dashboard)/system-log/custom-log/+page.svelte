<script>
  import { logChannel } from 'stores/sock';
  import CustomLog from './CustomLog.svelte';
</script>

<div class="p-5">
  <div class="flex space-x-5 py-5">
    <h1 class="text-3xl tracking-widest">Custom Log</h1>
  </div>
  <hr />
  <br />
  <CustomLog {logChannel} />
</div>
