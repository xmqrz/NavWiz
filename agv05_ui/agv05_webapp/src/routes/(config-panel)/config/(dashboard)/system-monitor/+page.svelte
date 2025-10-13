<script>
  import { title } from 'stores/page-title';
  import { page } from '$app/stores';
  import { afterNavigate } from '$app/navigation';
  import { onMount, onDestroy } from 'svelte';

  import monitor from '$lib/shared/services/config/system-monitor';

  export let data;

  title.set('System Monitor');

  afterNavigate(async ({ from }) => {
    // if refresh (click the same link)
    if ($page.route.id === from?.route.id) {
      let newData = await monitor.get();
      if (newData) {
        data.usage = newData;
      }
    }
  });

  const POLL_MS = 1000;
  let _timer;

  async function poll() {
    try {
      const usage = await monitor.get();

      // calculate CPU usage
      let diff = Object.fromEntries(
        Object.keys(usage.cpu_times).map((k) => [
          k,
          Math.max(0, usage.cpu_times[k] - data.usage.cpu_times[k])
        ])
      );
      let total = Object.values(diff).reduce((sum, value) => sum + value, 0);
      let cpu = (((total - diff.idle - diff.iowait) / total) * 100).toFixed(3) + '%';

      data.usage = usage;
      data.usage.cpu = cpu;
    } catch (e) {
      // ignore transient fetch errors during polling
    } finally {
      if (_timer) {
        _timer = setTimeout(poll, POLL_MS);
      }
    }
  }

  onMount(() => (_timer = setTimeout(poll, POLL_MS)));
  onDestroy(() => {
    clearTimeout(_timer);
    _timer = null;
  });
</script>

<div class="p-5">
  <div class="flex space-x-5 py-5">
    <h1 class="text-3xl tracking-widest">System Monitor</h1>
  </div>
  <hr />
  <br />
  <div class="w-full">
    <div>
      <p>CPU Usage: {data.usage.cpu ?? 'No data'}</p>
      <br />
      <p>CPU Freq: {data.usage.cpu_freq ?? 'No data'}</p>
      <br />
      <p>Memory Usage:</p>
      <pre class="card my-2 p-4">{data.usage.memory ?? 'No data'}</pre>
      <br />
      <p>Hard Disk Usage:</p>
      <pre class="card my-2 p-4">{data.usage.disk ?? 'No data'}</pre>
    </div>
  </div>
</div>
