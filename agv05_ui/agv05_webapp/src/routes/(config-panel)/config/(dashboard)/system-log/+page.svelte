<script>
  import { title } from 'stores/page-title';

  import LogDownload from './LogDownload.svelte';

  title.set('System Log');
</script>

<div class="p-5">
  <div class="flex space-x-5 py-5">
    <h1 class="text-3xl tracking-widest">System Log</h1>
  </div>
  <hr />
  <br />
  <nav class="list-nav">
    <ul class="md:max-w-lg">
      <li>
        <a href="/config/system-log/main-logs">
          <span class="badge bg-gray-500 text-white">
            <i class="fa-solid fa-arrow-right"></i>
          </span>
          <span class="flex-auto">Main log files</span>
        </a>
      </li>
      <li>
        <a href="/config/system-log/peripheral-logs">
          <span class="badge bg-gray-500 text-white">
            <i class="fa-solid fa-arrow-right"></i>
          </span>
          <span class="flex-auto">Peripheral log files</span>
        </a>
      </li>
      <li>
        <a href="/config/system-log/custom-log">
          <span class="badge bg-gray-500 text-white">
            <i class="fa-solid fa-arrow-right"></i>
          </span>
          <span class="flex-auto">Custom log</span>
        </a>
      </li>
      <li>
        <a href="/config/system-log/diagnostics">
          <span class="badge bg-gray-500 text-white">
            <i class="fa-solid fa-arrow-right"></i>
          </span>
          <span class="flex-auto">Diagnostics</span>
        </a>
      </li>
    </ul>
  </nav>
  <LogDownload />
</div>
