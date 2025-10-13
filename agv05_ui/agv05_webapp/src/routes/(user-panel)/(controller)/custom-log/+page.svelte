<script>
  import { appLogChannel } from 'stores/sock';
  import CustomLog from '../../../(config-panel)/config/(dashboard)/system-log/custom-log/CustomLog.svelte';
</script>

<CustomLog logChannel={appLogChannel} downloadable={false} />
